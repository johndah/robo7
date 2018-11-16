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
	//Server
	ros::ServiceServer update_map_server;
	//Client
  ros::ServiceClient ransac_srv;
	//Publisher
  ros::Publisher former_map_pub;

	UpdateMap()
	{
		//ServiceServer
		// update_map_server = n.advertiseService("/localization/mapping/update_map", &UpdateMap::update_Sequence, this);

		//Service Clients
		// ransac_srv = n.serviceClient<robo7_srvs::RansacWall>("/localization/ransac");

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
		robot_pose = req.the_robot_pose.position;
		lidar_scan = req.the_robot_pose.the_lidar_scan;
		extract_previous_grid();

		//Create a new matrix that can contains all of the lidar scan measures
		//and previous occupancy grid map. All it cells should be seen as
		//unoccupied cells
		create_current_pose_grid();
		create_new_sized_matrix();

		//Stick the previous occupancy grid on the top of the new grid at its
		//right position

		//Define a window around the robot where every single cell is unknown
		//state and plot the lidar point as occupied and the straight line
		//between the robot and lidar point as unoccupied
		fill_up_the_local_grid_with_lidar_scan();

		//Stick this new grid on top of the current grid at its right position.
		//Keep in mind that "unknown cells" should take tthe value of the previous
		//grid cells

		//Then return the newly defined grid so that it can be saved in the brain


		return true;
	}

private:
	//Input and output of the main service


	void create_new_sized_grid()
	{
		//The matrix is defined as :
		//  -> growing in column : the x_axis
		//  -> growing in row : the y_axis
		//The top left corner of the grid is always seen as a known position in
		//terms of (x,y) and the rest of the cells are defined based on this cell

		//Let's find the new size of the grid
		x_current_min = std::min(x_local_min, x_prev_min);
		x_current_max = std::max(x_local_max, x_prev_max);
		y_current_min = std::min(y_local_min, y_prev_min);
		y_current_max = std::max(y_local_max, y_prev_max);

		//Define the new dimension of the matrix
		current_nb_rows = (int)((x_current_max - x_current_min) / cell_size) + 1;
		current_nb_cols = (int)((y_current_max - y_current_min) / cell_size) + 1;

		//Create a large enough matrix that can contain all the cells in
		current_grid = Eigen::MatrixXf::Zero(current_nb_rows, current_nb_cols);
	}

	void create_current_pose_grid()
	{
		//The window around the robot is always the same size, only the corresponding
		//(x,y) of each cells is moving along with the robot
		the_local_grid = Eigen::MatrixXf::Zero(local_window_size, local_window_size);
		x_local_min = robot_pose.position - cell_size * the_local_grid.cols() / 2;
		x_local_max = x_local_min + cell_size * the_local_grid.cols();
		y_local_max = robot_pose.position + cell_size * the_local_grid.rows() / 2;
		y_local_min = y_local_max - cell_size * extract_grid.rows();
	}

	void extract_previous_grid()
	{
		prev_grid.resize(extract_grid.nb_rows, extract_grid.nb_cols);
		for(int i=0; i < extract_grid.nb_rows; i++)
		{
			for(int j=0; j < extract_grid.nb_cols; j++)
			{
				prev_grid(i,j) = extract_grid.rows[i].cols[j];
			}
		}
		x_prev_min = prev_grid.top_left_corner.x;
		x_prev_max = x_prev_max + cell_size * extract_grid.nb_cols;
		y_prev_max = prev_grid.top_left_corner.y;
		y_prev_min = y_prev_max - cell_size * extract_grid.nb_rows;
	}

	void fill_up_the_local_grid_with_lidar_scan()
	{
		//First, you need to to convert the lidar scan into a classic coordinates
		//scan with the appropriate service
		robo7_srvs::scanCoord::Request req1;
    robo7_srvs::scanCoord::Response res1;
    req1.robot_position = robot_pose.position;
    req1.lidar_scan = lidar_scan;
    scan_to_coord_srv.call(req1, res1);
		classic_coordinates_lidar_scan = res1.the_lidar_points;

		//Then, you need to fill up the grid cells with it
		for(int i=0; i < classic_coordinates_lidar_scan.number; i++)
		{
			x_lidar = classic_coordinates_lidar_scan.the_points[i].x;
			y_lidar = classic_coordinates_lidar_scan.the_points[i].y;
			row_cell = (int)(cell_size/2 + (y_local_max - y_lidar)/cell_size);
			col_cell = (int)(cell_size/2 + (x_lidar - x_local_min)/cell_size);
		}
	}

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
