#include <algorithm>
#include <math.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/explore.h"
#include "robo7_msgs/XY_coordinates.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <image_transport/image_transport.h>
#include "robo7_msgs/grid_matrix.h"
#include "robo7_msgs/grid_row.h"

typedef std::vector<float> Array;
typedef std::vector<Array> Matrix;

float pi = 3.14159265358979323846;

class MappingGridsServer
{
  public:
	ros::NodeHandle n;
	ros::Subscriber map_sub;
	ros::Publisher occupancy_pub, exploration_pub;
	robo7_msgs::grid_matrix grid_matrix_msg, exoploration_matrix_msg;
	ros::ServiceServer is_occupied_service;
	ros::ServiceServer explore_service;
	Matrix grid;

	MappingGridsServer()
	{
		// Parameters
		n.param<float>("/mapping_grids_server/grid_square_size", grid_square_size, 0.02);
		n.param<float>("/mapping_grids_server/min_distance", min_distance, 0.13);
		n.param<int>("/mapping_grids_server/smoothing_kernel_size", smoothing_kernel_size, 15);
		n.param<int>("/mapping_grids_server/smoothing_kernel_sd", smoothing_kernel_sd, 3);

		map_sub = n.subscribe("/own_map/wall_coordinates", 1, &MappingGridsServer::mapCallback, this);
		is_occupied_service = n.advertiseService("/occupancy_grid/is_occupied", &MappingGridsServer::occupancyGridRequest, this);
		explore_service = n.advertiseService("/exploration_grid/explore", &MappingGridsServer::explorationGridRequest, this);

		occupancy_pub = n.advertise<robo7_msgs::grid_matrix>("/mapping_grids_server/grid_matrix", 1);
		exploration_pub = n.advertise<robo7_msgs::grid_matrix>("/mapping_grids_server/exploration_matrix", 1);

		num_min_distance_squares = ceil(min_distance / grid_square_size);

		if (smoothing_kernel_size % 2 == 0)
		{
			smoothing_kernel_size += 1;
			ROS_WARN("Entered kernel size is even number, changing to: %d", smoothing_kernel_size);
		}

		window_width = .3;
		window_height = .2;
	}

	void mapCallback(const robo7_msgs::XY_coordinates::ConstPtr &msg)
	{
		X_wall_coordinates = msg->X_coordinates;
		Y_wall_coordinates = msg->Y_coordinates;
	}

	int sq(float coord)
	{
		return floor(coord / grid_square_size);
	}

	bool explorationGridRequest(robo7_srvs::explore::Request &req,
								robo7_srvs::explore::Response &res)
	{
		ROS_DEBUG("New grid exploration request recieved");

		float x = req.x;
		float y = req.y;
		float theta = req.theta;

		res.explored = getExplorationGrid(x, y, theta);

		return true;
	}

	bool occupancyGridRequest(robo7_srvs::IsGridOccupied::Request &req,
							  robo7_srvs::IsGridOccupied::Response &res)
	{

		ROS_DEBUG("New grid occupancy request recieved");

		if (sq(req.x) >= num_grid_squares_x || sq(req.y) >= num_grid_squares_y)
		{
			res.occupancy = 1.0;
		}
		else
		{
			float value = occupancy_grid.at<float>(sq(req.x), sq(req.y));

			if (value < 0.0001)
			{
				res.occupancy = 0.0;
			}
			else
			{
				res.occupancy = value;
			}
		}

		if (!occupancy_grid_init)
		{
			grid_matrix_msg = publishOccupancyGrid();

			for (int i = 0; i < 10; i++)
				occupancy_pub.publish(grid_matrix_msg);

			occupancy_grid_init = true;
		}

		return true;
	}

	void updateBasicGridSize()
	{
		ROS_DEBUG("Setting mapping grids size");

		// Wait untill we get our first coordinates
		while (X_wall_coordinates.size() <= 0)
		{
			ros::spinOnce();
		}

		// Get maximum coordinates in grid
		float x_max = *max_element(X_wall_coordinates.begin(), X_wall_coordinates.end());
		float y_max = *max_element(Y_wall_coordinates.begin(), Y_wall_coordinates.end());
		ROS_DEBUG("x_max: %f", x_max);
		ROS_DEBUG("y_max: %f", y_max);

		num_grid_squares_x = ceil(x_max / grid_square_size);
		num_grid_squares_y = ceil(y_max / grid_square_size);
		ROS_DEBUG("x squares: %d", num_grid_squares_x);
		ROS_DEBUG("y squares: %d", num_grid_squares_y);

		// Set up sizes
		grid.resize(num_grid_squares_x);
		for (int i = 0; i < num_grid_squares_x; ++i)
			grid[i].resize(num_grid_squares_y);

		updateBasicGrid();
	}

	float distance_points(int x, int y, int a, int b)
	{
		float x_diff = x - a;
		float y_diff = y - b;
		//ROS_INFO("x: %d, y: %d, a: %d, b: %d", x, y, a, b);
		return std::sqrt(x_diff * x_diff + y_diff * y_diff);
	}

	void updateBasicGrid()
	{
		ROS_DEBUG("Updating occupancy grid");

		// Put every point in the grid
		for (int i = 0; i < X_wall_coordinates.size(); ++i)
		{
			// Get the square that the point is located in
			int square_x = sq(X_wall_coordinates[i]);
			int square_y = sq(Y_wall_coordinates[i]);

			int x_low = square_x - num_min_distance_squares;
			int y_low = square_y - num_min_distance_squares;

			int x_high = square_x + num_min_distance_squares;
			int y_high = square_y + num_min_distance_squares;

			// Setting 1.0 for all the walls (enlarged by the min distance)
			for (int i = x_low; i <= x_high; ++i)
			{
				for (int j = y_low; j <= y_high; ++j)
				{
					if (!(i < 0) && !(i >= num_grid_squares_x) && !(j < 0) && !(j >= num_grid_squares_y))
					{
						// This keeps the wall expansion radial
						if (distance_points(square_x, square_y, i, j) <= (num_min_distance_squares))
						{
							grid[i][j] = 1.0;
						}
					}
				}
			}
		}

		// Setting the filtered grid

		cv::Mat basic_grid(grid.size(), grid.at(0).size(), CV_64FC1);

		for (int i = 0; i < basic_grid.rows; ++i)
		{
			for (int j = 0; j < basic_grid.cols; ++j)
			{
				basic_grid.at<float>(i, j) = grid[i][j];
			}
		}

		occupancy_grid = gaussFilter(basic_grid, smoothing_kernel_size, smoothing_kernel_sd);
	}

	cv::Mat gaussFilter(cv::Mat grid_in, int kernel_size, int sigma)
	{
		cv::Mat grid_filtered;

		GaussianBlur(grid_in, grid_filtered, cv::Size(kernel_size, kernel_size), sigma, 0);

		cv::Mat normalized_grid;
		cv::normalize(grid_filtered, normalized_grid, 0, 1, cv::NORM_MINMAX, CV_32F);

		cv::Mat normalized_grid_ones = normalized_grid.clone();

		for (int i = 0; i < normalized_grid.rows; ++i)
		{
			for (int j = 0; j < normalized_grid.cols; ++j)
			{
				// if(unfiltered_grid[i][j] >= 1){
				if (grid[i][j] >= 1)
				{
					normalized_grid_ones.at<float>(i, j) = 1;
				}
			}
		}

		//DISPLAY: Uncomment here and
		// namedWindow("Display window", cv::WINDOW_NORMAL );
		// cv::resizeWindow("Display window", 600,600);
		// imshow( "Display window", grid_in );
		// cv::waitKey(0);
		// imshow( "Display window", grid_filtered );
		// cv::waitKey(0);
		// imshow( "Display window", normalized_grid );
		// cv::waitKey(0);
		// imshow( "Display window", normalized_grid_ones );
		// cv::waitKey(0);
		// cvDestroyWindow("Display window");

		ROS_INFO("Gaussed grid ready");
		return normalized_grid_ones;
	}

	int getExplorationGrid(float x, float y, float theta)
	{
		if (!exploration_grid_init)
		{
			exploration_grid = cv::Mat::zeros(num_grid_squares_x, num_grid_squares_y, CV_32SC1);
			exploration_grid_init = true;
		}

		theta = pi / 2 - theta;

		float i0 = x + window_width * cos(theta) / 2.0;
		float j0 = y - window_width * sin(theta) / 2.0;

		float i_inc = float(cos(theta) * grid_square_size) / 2.0;
		float j_inc = float(sin(theta) * grid_square_size) / 2.0;
		float j_max = float(window_height / grid_square_size);
		float x_grid, y_grid, i_shift, i_max;

		for (float j = 0.0; j < j_max; j += .5)
		{
			i_shift = float(window_height / grid_square_size) / 2 - j / 2;
			i_max = float(window_width / grid_square_size) - i_shift;

			for (float i = i_shift; i < i_max; i += .5)
			{
				x_grid = i0 + grid_square_size * (j * sin(theta) - i * cos(theta));
				y_grid = j0 + grid_square_size * (j * cos(theta) + i * sin(theta));

				if (exploration_grid.at<float>(sq(x_grid), sq(y_grid)) < 1.0 && !(sq(x_grid) < 0) && !(sq(x_grid) >= num_grid_squares_x) && !(sq(y_grid) < 0) && !(sq(y_grid) >= num_grid_squares_y))
				{
					if (j < 1.0 || j > j_max - 1.5 || i < i_shift + 1.0 || i > i_max - 1.5)
						exploration_grid.at<float>(sq(x_grid), sq(y_grid)) = -1.0;
					else
						exploration_grid.at<float>(sq(x_grid), sq(y_grid)) = 1.0;
				}
			}
		}

		grid_matrix_msg = publishExplorationGrid();

		for (int i = 0; i < 10; i++)
			exploration_pub.publish(grid_matrix_msg);

		return exploration_grid.at<int>(sq(x), sq(y));
	}

	robo7_msgs::grid_matrix publishOccupancyGrid()
	{
		robo7_msgs::grid_row grid_row_msg;
		robo7_msgs::grid_matrix grid_matrix_msg;

		std::vector<float> grid_row;

		for (int i = 0; i < occupancy_grid.rows; i++)
		{
			grid_row.clear();
			for (int j = 0; j < occupancy_grid.cols; j++)
			{
				grid_row.push_back(occupancy_grid.at<float>(i, j));
			}
			grid_row_msg.grid_row = grid_row;
			grid_matrix_msg.grid_rows.push_back(grid_row_msg);
		}

		return grid_matrix_msg;
	}

	robo7_msgs::grid_matrix publishExplorationGrid()
	{
		robo7_msgs::grid_row exploration_row_msg;
		robo7_msgs::grid_matrix exploration_matrix_msg;

		std::vector<float> exploration_row;

		for (int i = 0; i < exploration_grid.rows; i++)
		{
			exploration_row.clear();
			for (int j = 0; j < exploration_grid.cols; j++)
			{
				exploration_row.push_back(exploration_grid.at<float>(i, j));
			}
			exploration_row_msg.grid_row = exploration_row;
			exploration_matrix_msg.grid_rows.push_back(exploration_row_msg);
		}

		return exploration_matrix_msg;
	}

  private:
	float min_distance;
	int num_min_distance_squares;
	float grid_square_size;
	int smoothing_kernel_size;
	int smoothing_kernel_sd;
	int num_grid_squares_x;
	int num_grid_squares_y;
	float window_width;
	float window_height;
	std::vector<float> X_wall_coordinates;
	std::vector<float> Y_wall_coordinates;
	cv::Mat basic_grid;
	cv::Mat occupancy_grid;
	cv::Mat exploration_grid;
	float current_x_to, current_y_to;
	bool occupancy_grid_init, exploration_grid_init;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapping_grids_server");

	MappingGridsServer mapping_grids_server;

	ros::Rate loop_rate(100);

	ROS_INFO("Mapping grids server running");

	mapping_grids_server.updateBasicGridSize();

	ros::spin();

	return 0;
}
