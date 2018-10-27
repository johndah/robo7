#include <algorithm>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "robo7_srvs/IsGridOccupied.h"
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
		n.param<double>("/occupancy_grid_server/min_distance", min_distance, 0.13);
		n.param<int>("/occupancy_grid_server/smoothing_kernel_size", smoothing_kernel_size, 3);
		n.param<double>("/occupancy_grid_server/grid_square_size", grid_square_size, 0.02);

		map_sub = n.subscribe("/own_map/wall_coordinates", 1, &OccupancyGridServer::mapCallback, this);
		is_occupied_service = n.advertiseService("/occupancy_grid/is_occupied", &OccupancyGridServer::gridRequest, this);

		num_min_distance_squares = ceil(min_distance/grid_square_size);

	}

	void mapCallback(const robo7_msgs::XY_coordinates::ConstPtr &msg)
	{
		X_wall_coordinates = msg->X_coordinates;
		Y_wall_coordinates = msg->Y_coordinates;
	}

	void updateGridSize()
	{
		ROS_INFO("Setting grid size");

		// Wait untill we get our first coordinates
		while (X_wall_coordinates.size() <= 0){
			ros::spinOnce();
		}

		// Get maximum coordinates in grid
		float x_max = *max_element(X_wall_coordinates.begin(), X_wall_coordinates.end());
		float y_max = *max_element(Y_wall_coordinates.begin(), Y_wall_coordinates.end());
		ROS_INFO("x_max: %f", x_max);
		ROS_INFO("y_max: %f", y_max);

		num_grid_squares_x = ceil(x_max/grid_square_size);
		num_grid_squares_y = ceil(y_max/grid_square_size);
		ROS_INFO("x squares: %d", num_grid_squares_x);
		ROS_INFO("y squares: %d", num_grid_squares_y);

		// Set up sizes
		grid.resize(num_grid_squares_x);
	  for (int i = 0; i < num_grid_squares_x; ++i)
		    grid[i].resize(num_grid_squares_y);

		updateGrid();
	}


	void updateGrid()
	{
		ROS_INFO("Updating grid");

		// Put every point in the grid
		for (int i = 0; i< X_wall_coordinates.size(); ++i){
			// Get the square that the point is located in
			int square_x = floor(X_wall_coordinates[i]/grid_square_size);
			int square_y = floor(Y_wall_coordinates[i]/grid_square_size);

			int x_low = square_x - num_min_distance_squares;
			int y_low = square_y - num_min_distance_squares;

			int x_high = square_x + num_min_distance_squares;
			int y_high = square_y + num_min_distance_squares;

			for (int i = x_low; i <= x_high ; ++i){
	 		  for (int j = y_low; j <= y_high; ++j) {
					if (!(i < 0) && !(i >= num_grid_squares_x) && !(j < 0) && !(j >= num_grid_squares_y))
					{
						grid[i][j] = 1.0;
					}
				}
			}


			//TODO: Gaussian filter here

		}

	}

	void printGrid(){
		// Print grid, only for testing
		for (int j = num_grid_squares_y-1; j >= 0; --j) {
			std::string stra = " ";
	 		for (int i = 0; i < num_grid_squares_x; ++i){
				double value = grid[i][j];
				if (value > 0){
					stra += "1";
				}
				else{
					stra += "0";
				}
			}
			ROS_INFO(stra.c_str());
	 	}


	}

	bool gridRequest(robo7_srvs::IsGridOccupied::Request &req,
         robo7_srvs::IsGridOccupied::Response &res)
	{
	  ROS_INFO("New grid occupancy request recieved");

		// only need to update grid when we actually change the map
		//updateGrid();

		int req_x_grid = floor(req.x/grid_square_size);
		int req_y_grid = floor(req.y/grid_square_size);

		if (req_x_grid >= num_grid_squares_x || req_y_grid >= num_grid_squares_y){
			res.occupancy = 1.0;
		}
		else{
			res.occupancy = grid[req_x_grid][req_y_grid];
		}

    return true;
	}

private:
	double min_distance;
	int num_min_distance_squares;
	int smoothing_kernel_size;
	double grid_square_size;
	int num_grid_squares_x;
	int num_grid_squares_y;
	std::vector<float> X_wall_coordinates;
	std::vector<float> Y_wall_coordinates;
	std::vector<std::vector<double> > grid;


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "occupancy_grid_server");

	OccupancyGridServer occupancy_grid_server;

	ros::Rate loop_rate(10);

	ROS_INFO("Occupancy grid server running");

	occupancy_grid_server.updateGridSize();
	occupancy_grid_server.printGrid();

	ros::spin();

	return 0;
}
