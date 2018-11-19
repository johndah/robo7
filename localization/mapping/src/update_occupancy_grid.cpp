#include <ros/ros.h>
#include <Eigen/Geometry>

//The messages
#include <geometry_msgs/Twist.h>
#include <robo7_msgs/mapping_grid.h>
#include <sensor_msgs/LaserScan.h>

//The services
#include <robo7_srvs/UpdateOccupancyGrid.h>
#include <robo7_srvs/scanCoord.h>

#include <stdlib.h>

#include <string>
#include <vector>


class Update_Occ_Grid
{
public:
	ros::NodeHandle n;
	//Server
	ros::ServiceServer update_occupancy_server;
	//Client
  ros::ServiceClient scan_to_coord_srv;
	//Publisher
  ros::Publisher local_grid_pub;
	ros::Publisher updated_grid_pub;

	Update_Occ_Grid()
	{
		//Input parameters definition of the node
		n.param<float>("/update_occupancy_grid/local_window_size", local_window_size, 1.05);
		n.param<float>("/update_occupancy_grid/cell_size", cell_size, 0.05);

		//Occupancy grid values Initialisation
		unknown = 0;
		unoccupied = 1;
		occupied = 2;

		//ServiceServer
		update_occupancy_server = n.advertiseService("/localization/mapping/update_occupancy_grid", &Update_Occ_Grid::update_Sequence, this);

		//Service Clients
		scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");

		//Visualization of the updated versions of the walls
		local_grid_pub = n.advertise<robo7_msgs::mapping_grid>("/mapping/local_occupancy_grid", 1);
		updated_grid_pub = n.advertise<robo7_msgs::mapping_grid>("/mapping/updated_occupancy_grid", 1);
	}

	bool update_Sequence(robo7_srvs::UpdateOccupancyGrid::Request &req,
         robo7_srvs::UpdateOccupancyGrid::Response &res)
	{
		//Extract the pose_timed out of the request message
		//Both lidar scan and corresponding position
		robot_pose = req.the_robot_pose.position;
		lidar_scan = req.the_robot_pose.the_lidar_scan;
		extract_grid_msg = req.occupancy_grid;
		extract_previous_grid();

		//Create a new matrix that can contains all of the lidar scan measures
		//and previous occupancy grid map. All it cells should be seen as
		//unoccupied cells
		// ROS_INFO("Create Local map grid");
		create_local_pose_grid();
		create_new_sized_grid();


		//Stick the previous occupancy grid on the top of the new grid at its
		//right position
		stick_previous_occupancy_grid();

		//Define a window around the robot where every single cell is unknown
		//state and plot the lidar point as occupied and the straight line
		//between the robot and lidar point as unoccupied
		// ROS_INFO("Fill up the local grid with lidar scan");
		fill_up_the_local_grid_with_lidar_scan();

		//Stick this new grid on top of the current grid at its right position.
		//Keep in mind that "unknown cells" should take tthe value of the previous
		//grid cells
		update_current_occupancy_grid_with_local_grid();

		// ROS_INFO("Local (x,y) : (%lf, %lf)", x_local_min, y_local_max);
		// ROS_INFO("Current (x,y) : (%lf, %lf)", x_current_min, y_current_max);

		//Then fill up the mapping grid with the corresponding values
		// ROS_INFO("Fill up the messages to send");
		fill_up_the_messages();

	  local_grid_pub.publish( local_grid_msg );
		updated_grid_pub.publish( updated_grid_msg );

		//Return / Publish the messages
		res.updated_occupancy_grid = updated_grid_msg;

		return true;
	}

private:
	//Input and output of the main service
	geometry_msgs::Twist robot_pose;
	sensor_msgs::LaserScan lidar_scan;
	robo7_msgs::mapping_grid extract_grid_msg;
	robo7_msgs::mapping_grid updated_grid_msg;

	//The publish messages
	robo7_msgs::mapping_grid local_grid_msg;


	//The local grid parameters
	Eigen::MatrixXf the_local_grid;
	float x_local_min, x_local_max;
	float y_local_min, y_local_max;
	robo7_msgs::wallPoint classic_coordinates_lidar_scan;

	//The previous grid
	Eigen::MatrixXf the_prev_grid;
	float x_prev_min, x_prev_max;
	float y_prev_min, y_prev_max;

	//The current grid grid
	Eigen::MatrixXf the_current_grid;
	float x_current_min, x_current_max;
	float y_current_min, y_current_max;
	int current_nb_rows, current_nb_cols;

	//The different grid values
	float unknown, occupied, unoccupied;
	float local_window_size, cell_size;

	//Usefull global variable
	float x_robot, y_robot;
	float x_lidar, y_lidar;

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
		current_nb_cols = (int)((x_current_max - x_current_min) / cell_size);
		current_nb_rows = (int)((y_current_max - y_current_min) / cell_size);

		//Create a large enough matrix that can contain all the cells in
		the_current_grid.resize(current_nb_rows, current_nb_cols);
		the_current_grid = Eigen::MatrixXf::Zero(current_nb_rows, current_nb_cols);
	}

	void create_local_pose_grid()
	{
		//The window around the robot is always the same size, only the corresponding
		//(x,y) of each cells is moving along with the robot
		the_local_grid = Eigen::MatrixXf::Zero((int)local_window_size/cell_size, (int)local_window_size/cell_size);
		x_local_min = robot_pose.linear.x - cell_size * the_local_grid.cols() / 2;
		x_local_max = x_local_min + cell_size * the_local_grid.cols();
		y_local_max = robot_pose.linear.y + cell_size * the_local_grid.rows() / 2;
		y_local_min = y_local_max - cell_size * the_local_grid.rows();
	}

	void extract_previous_grid()
	{
		cell_size = extract_grid_msg.cell_size;
		the_prev_grid.resize(extract_grid_msg.occupancy_grid.nb_rows, extract_grid_msg.occupancy_grid.nb_cols);
		for(int i=0; i < extract_grid_msg.occupancy_grid.nb_rows; i++)
		{
			for(int j=0; j < extract_grid_msg.occupancy_grid.nb_cols; j++)
			{
				the_prev_grid(i,j) = extract_grid_msg.occupancy_grid.rows[i].cols[j];
			}
		}
		x_prev_min = extract_grid_msg.top_left_corner.x;
		x_prev_max = x_prev_min + cell_size * extract_grid_msg.occupancy_grid.nb_cols;
		y_prev_max = extract_grid_msg.top_left_corner.y;
		y_prev_min = y_prev_max - cell_size * extract_grid_msg.occupancy_grid.nb_rows;
	}

	void stick_previous_occupancy_grid()
	{
		std::vector<int> start_cell = corresponding_global_cell(x_prev_min, y_prev_max);
		int start_x_ind = start_cell[0];
		int start_y_ind = start_cell[1];

		for(int i = 0; i < the_prev_grid.rows(); i++)
		{
			for(int j = 0; j < the_prev_grid.cols(); j++)
			{
				the_current_grid(start_x_ind + i, start_y_ind + j) = the_prev_grid(i,j);
			}
		}
	}

	void fill_up_the_local_grid_with_lidar_scan()
	{
		//First, you need to to convert the lidar scan into a classic coordinates
		//scan with the appropriate service
		robo7_srvs::scanCoord::Request req1;
    robo7_srvs::scanCoord::Response res1;
    req1.robot_position = robot_pose;
    req1.lidar_scan = lidar_scan;
    scan_to_coord_srv.call(req1, res1);
		classic_coordinates_lidar_scan = res1.the_lidar_points;

		//Then, you need to fill up the grid cells with it
		for(int i=0; i < classic_coordinates_lidar_scan.number; i++)
		{
			x_lidar = classic_coordinates_lidar_scan.the_points[i].x;
			y_lidar = classic_coordinates_lidar_scan.the_points[i].y;

			//Then for this specific cell, you need to mark it as occupied and the
			//others in between as unoccupied
			// ROS_INFO("Fill up one line");
			fill_a_local_line();
		}
	}

	void fill_a_local_line()
	{
    std::vector<int> cell_ind(2, 0);
		cell_ind = corresponding_local_cell(x_lidar, y_lidar);
		x_robot = robot_pose.linear.x;
		y_robot = robot_pose.linear.y;
		// ROS_INFO("(x_robot, y_robot) = (%lf, %lf), (x_lidar, y_lidar) = (%lf, %lf)", x_robot, y_robot, x_lidar, y_lidar);
		// ROS_INFO("Distance robot-lidar: (%lf, %lf)", x_robot-x_lidar, y_robot-y_lidar);
		// ROS_INFO("Inner cell : (%d, %d)", cell_ind[0], cell_ind[1]);
		// ROS_INFO("Matrix size : (%d, %d)", the_local_grid.rows(), the_local_grid.cols());
		if((0 <= cell_ind[0])&&(cell_ind[0] < the_local_grid.rows())
					&&(0 <= cell_ind[1])&&(cell_ind[1] < the_local_grid.cols()))
		{
			// ROS_INFO("Condition respected");
			int N_step = floor( sqrt(pow(x_robot - x_lidar,2) + pow(y_robot - y_lidar,2))/cell_size ) + 1;
			float x_step = (x_robot - x_lidar)/N_step;
			float y_step = (y_robot - y_lidar)/N_step;
			// ROS_INFO("Nb of step : %d, %lf, %lf", N_step, x_step, y_step);
			for(int j=0; j < N_step + 1; j++)
			{
				// ROS_INFO("%d", j);
				cell_ind = corresponding_local_cell(x_robot - j * x_step, y_robot - j * y_step);
				// ROS_INFO("cell to fill : (%d,%d)", cell_ind[0], cell_ind[1]);
				the_local_grid(cell_ind[0], cell_ind[1]) = unoccupied;
			}
			the_local_grid(cell_ind[0], cell_ind[1]) = occupied;
		}
		// ROS_INFO("_");
	}

	std::vector<int> corresponding_local_cell(float x, float y)
	{
		int row_cell = (int)(cell_size/2 + (y_local_max - y)/cell_size);
		int col_cell = (int)(cell_size/2 + (x - x_local_min)/cell_size);

		std::vector<int> cell_index(2, 0);
		cell_index[0] = row_cell;
		cell_index[1] = col_cell;

		return cell_index;
	}

	std::vector<int> corresponding_global_cell(float x, float y)
	{
		int row_cell = (int)(cell_size/2 + (y_current_max - y)/cell_size);
		int col_cell = (int)(cell_size/2 + (x - x_current_min)/cell_size);

		std::vector<int> cell_index(2, 0);
		cell_index[0] = row_cell;
		cell_index[1] = col_cell;

		return cell_index;
	}

	void update_current_occupancy_grid_with_local_grid()
	{
		std::vector<int> start_cell = corresponding_global_cell(x_local_min, y_local_max);
		int start_x_ind = start_cell[0];
		int start_y_ind = start_cell[1];

		// ROS_INFO("Start cell (i,j) : (%d, %d)", start_cell[0], start_cell[1]);
		for(int i = 0; i < the_local_grid.rows(); i++)
		{
			// ROS_INFO("Row i : (%d)", i);
			for(int j = 0; j < the_local_grid.cols(); j++)
			{
				// ROS_INFO("Checking cell (i,j) : (%d, %d)", start_x_ind + i, start_y_ind + j);
				if(the_local_grid(i,j) != unknown)
				{
					the_current_grid(start_x_ind + i, start_y_ind + j) = the_local_grid(i,j);
				}
			}
		}
	}

	void fill_up_the_messages()
	{
		//Fill up the local grid msg
		robo7_msgs::matrix local_matrix_msg;
		for(int i=0; i < the_local_grid.rows(); i++)
		{
			robo7_msgs::matrix_row one_row;
			for(int j=0; j < the_local_grid.cols(); j++)
			{
				one_row.cols.push_back( the_local_grid(i,j) );
			}
			local_matrix_msg.rows.push_back( one_row );
		}
		local_matrix_msg.nb_rows = the_local_grid.rows();
		local_matrix_msg.nb_cols = the_local_grid.cols();

		//Fill up the message
		local_grid_msg.top_left_corner.x = x_local_min;
		local_grid_msg.top_left_corner.y = y_local_max;
		local_grid_msg.window_width = local_window_size;
		local_grid_msg.window_height = local_window_size;
		local_grid_msg.cell_size = cell_size;
		local_grid_msg.occupancy_grid = local_matrix_msg;


		//Fill up the current grid msg
		robo7_msgs::matrix current_matrix_msg;
		for(int i=0; i < the_current_grid.rows(); i++)
		{
			robo7_msgs::matrix_row one_row;
			for(int j=0; j < the_current_grid.cols(); j++)
			{
				one_row.cols.push_back( the_current_grid(i,j) );
			}
			current_matrix_msg.rows.push_back( one_row );
		}
		current_matrix_msg.nb_rows = the_current_grid.rows();
		current_matrix_msg.nb_cols = the_current_grid.cols();

		//Fill up the message
		updated_grid_msg.top_left_corner.x = x_current_min;
		updated_grid_msg.top_left_corner.y = y_current_max;
		updated_grid_msg.window_width = current_matrix_msg.nb_rows * cell_size;
		updated_grid_msg.window_height = current_matrix_msg.nb_cols * cell_size;
		updated_grid_msg.cell_size = cell_size;
		updated_grid_msg.occupancy_grid = current_matrix_msg;
	}


};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Update occupancy service");

	Update_Occ_Grid Update_Occ_Grid_;

	ros::Rate loop_rate(100);

	ROS_INFO("Update Occupancy Service is running");

	ros::spin();

	return 0;
}
