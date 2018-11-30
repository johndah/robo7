#include <algorithm>
#include <cmath>
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
#include "robo7_srvs/distanceTo.h"

typedef std::vector<float> Array;
typedef std::vector<Array> Matrix;
float window_width, window_height;
int frontier_window_size, unexplored_threshold;

class Frontier;

typedef std::shared_ptr<Frontier> frontier_ptr;
float pi = 3.14159265358979323846;

class Frontier
{
  public:
	float x, y, occupancy_cost, theta_diff, frontier_distance, not_visable;
	int number_unexplored;
	//float angular_velocity, time, dt;
	//float path_cost, path_length;
	//float steering_angle_max, angular_velocity_resolution;
	//float tolerance_radius, tolerance_angle;
	//unsigned int node_id;
	//std::vector<float> path_x, path_y, path_theta;

	Frontier(float x, float y, float occupancy_cost, float theta_diff, float frontier_distance, int number_unexplored)
	{
		this->x = x;
		this->y = y;
		this->occupancy_cost = occupancy_cost;
		this->theta_diff = theta_diff;
		this->frontier_distance = frontier_distance;
		this->number_unexplored = number_unexplored;

		not_visable = false;
	}

	float getCost()
	{
		//ROS_INFO("Costs: occ %f, dist %f, dist_cost %f.  theta_diff %f  exp_cost %f", occupancy_cost, frontier_distance, getDistanceCost(frontier_distance, window_height), theta_diff / (4 * pi), getExplorationGainCost());

		return occupancy_cost + theta_diff / (8 * pi) + getExplorationGainCost() + 0 * getDistanceCost(frontier_distance, window_height) + float(not_visable);
	}

	float getExplorationGainCost()
	{
		float frontier_area = float(pow(2 * frontier_window_size, 2));
		// ROS_INFO("num explor %f   window2  % f    nom %f    denom %f", float(number_unexplored), frontier_area, float(frontier_area - float(number_unexplored)), frontier_area);
		return .2 * (frontier_area - float(number_unexplored)) / frontier_area;
		//return float(.8*(number_unexplored < unexplored_threshold));
	}

	float getDistanceCost(float distance, float threshold)
	{
		return distance / 6;

		// if (distance < threshold)
		// 	return 1 - gaussian(distance, threshold, threshold);
		// else
		// 	return 1 - std::exp(-(distance - threshold) * threshold);
	}

	float gaussian(float x, float mean, float var)
	{
		float diff = (x - mean) / var;
		return std::exp(-pow(diff, 2) / 2);
	}

	bool inCollision()
	{
		return occupancy_cost >= 1.0;
	}
};

class MappingGridsServer
{
  public:
	ros::NodeHandle n;
	ros::Subscriber map_sub;
	ros::Publisher occupancy_pub, wall_occupancy_pub, exploration_pub;
	robo7_msgs::grid_matrix grid_matrix_msg, exoploration_matrix_msg;
	ros::ServiceServer is_occupied_service;
	ros::ServiceServer explore_service;
	ros::ServiceClient occupancy_client, distance_client;
	robo7_srvs::distanceTo distance_srv;

	std::vector<frontier_ptr> all_frontiers_nodes;
	Matrix grid, wall_grid;
	bool get_frontier;

	MappingGridsServer()
	{
		// Parameters
		n.param<float>("/mapping_grids_server/grid_square_size", grid_square_size, 0.02);
		n.param<float>("/mapping_grids_server/min_distance", min_distance, 0.13);
		n.param<float>("/mapping_grids_server/wall_thickness", wall_thickness, 0.03);
		n.param<int>("/mapping_grids_server/smoothing_kernel_size", smoothing_kernel_size, 15);
		n.param<int>("/mapping_grids_server/smoothing_kernel_sd", smoothing_kernel_sd, 3);

		map_sub = n.subscribe("/own_map/wall_coordinates", 1, &MappingGridsServer::mapCallback, this);
		is_occupied_service = n.advertiseService("mapping_grids_server/occupancy_grid/is_occupied", &MappingGridsServer::occupancyGridRequest, this);
		explore_service = n.advertiseService("/exploration_grid/explore", &MappingGridsServer::explorationGridRequest, this);

		occupancy_pub = n.advertise<robo7_msgs::grid_matrix>("/mapping_grids_server/occupancy_matrix", 1);
		wall_occupancy_pub = n.advertise<robo7_msgs::grid_matrix>("/mapping_grids_server/wall_occupancy_matrix", 1);
		exploration_pub = n.advertise<robo7_msgs::grid_matrix>("/mapping_grids_server/exploration_matrix", 1);

		occupancy_client = n.serviceClient<robo7_srvs::IsGridOccupied>("mapping_grids_server/occupancy_grid/is_occupied");
		distance_client = n.serviceClient<robo7_srvs::distanceTo>("/distance_grid/distance");

		num_min_distance_squares = ceil(min_distance / grid_square_size);
		num_wall_thickness_squares = ceil(wall_thickness / grid_square_size);

		if (smoothing_kernel_size % 2 == 0)
		{
			smoothing_kernel_size += 1;
			ROS_WARN("Entered kernel size is even number, changing to: %d", smoothing_kernel_size);
		}

		window_width = .45;
		window_height = .45;
		unexplored_threshold = 25;
		frontier_window_size = 4;
	}

	bool explorationGridRequest(robo7_srvs::explore::Request &req, robo7_srvs::explore::Response &res)
	{
		ROS_DEBUG("New grid exploration request recieved");

		float x = req.x;
		float y = req.y;
		float theta = req.theta;
		get_frontier = req.get_frontier;
		distance_grid_init = false;

		geometry_msgs::Twist frontier_destination_pose = getFrontier(x, y, theta, res);

		res.frontier_destination_pose = frontier_destination_pose;
		res.success = true;

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

	geometry_msgs::Twist getFrontier(float x, float y, float theta, robo7_srvs::explore::Response &res)
	{

		std::vector<frontier_ptr> frontier_nodes;

		if (!exploration_grid_init)
		{
			exploration_grid = cv::Mat::zeros(num_grid_squares_x, num_grid_squares_y, CV_32SC1);

			grid_matrix_msg = publishWallGrid();

			for (int i = 0; i < 10; i++)
				wall_occupancy_pub.publish(grid_matrix_msg);

			exploration_grid_init = true;
		}

		theta = pi / 2 - theta;

		float i0 = x + window_width * cos(theta) / 2.0;
		float j0 = y - window_width * sin(theta) / 2.0;

		float i_inc = float(cos(theta) * grid_square_size) / 2.0;
		float j_inc = float(sin(theta) * grid_square_size) / 2.0;
		float j_max = float(window_height / grid_square_size);
		float x_grid, y_grid, x_ray, y_ray, i_shift, i_max;

		float x_frontier, y_frontier, frontier_distance, x_diff, y_diff, theta_diff;
		for (int i = 0; i < all_frontiers_nodes.size(); i++)
		{
			x_frontier = all_frontiers_nodes[i]->x;
			y_frontier = all_frontiers_nodes[i]->y;

			if (exploration_grid.at<float>(sq(x_frontier), sq(y_frontier)) == 1.0) // || number_unexplored < unexplored_threshold)
				all_frontiers_nodes.erase(all_frontiers_nodes.begin() + i);
			else
			{
				frontier_distance = distance(x, y, x_frontier, y_frontier);
				all_frontiers_nodes[i]->frontier_distance = frontier_distance;

				x_diff = float(x_frontier - x);
				y_diff = float(y_frontier - y);
				theta_diff = std::abs(std::fmod(theta - atan2(y_diff, x_diff) + pi, 2 * pi) - pi);
				all_frontiers_nodes[i]->theta_diff = theta_diff;

				int number_unexplored = 0;
				for (int i_frontier = sq(x_frontier) - frontier_window_size; i_frontier < sq(x_frontier) + frontier_window_size; i_frontier++)
				{
					for (int j_frontier = sq(y_frontier) - frontier_window_size; j_frontier < sq(y_frontier) + frontier_window_size; j_frontier++)
					{
						if (exploration_grid.at<float>(i_frontier, j_frontier) == 0.0)
						{
							number_unexplored++;
						}
					}
				}
				all_frontiers_nodes[i]->number_unexplored = number_unexplored;

				int n = 4 * std::max(std::max(std::abs(sq(x_diff)), std::abs(sq(y_diff))), 20);
				bool not_visable = false;
				// ROS_INFO("Mapping n %d", n);

				x_ray = x;
				y_ray = y;

				for (int i_ray = 0; i_ray < n; i_ray++)
				{

					x_ray += x_diff / n;
					y_ray += y_diff / n;

					if (grid[sq(x_ray)][sq(y_ray)] == 1.0)
					{
						not_visable = true;
						break;
					}
				}
				// ROS_INFO("Occupancy %f", grid[sq(x_ray)][sq(y_ray)]);
				all_frontiers_nodes[i]->not_visable = not_visable;
				/*
				*/
			}
		}

		bool add_exploration_cell;
		for (float j = 0.0; j < j_max; j += .5)
		{
			i_shift = float(window_height / grid_square_size) / 3 - j / 3;
			i_max = float(window_width / grid_square_size) - i_shift;

			for (float i = i_shift; i < i_max; i += .5)
			{
				x_grid = i0 + grid_square_size * (j * sin(theta) - i * cos(theta));
				y_grid = j0 + grid_square_size * (j * cos(theta) + i * sin(theta));

				add_exploration_cell = true;
				if (exploration_grid.at<float>(sq(x_grid), sq(y_grid)) < 1.0 && withinMap(x_grid, y_grid))
				{
					float frontier_distance = distance(x, y, x_grid, y_grid);
					if (frontier_distance > .1)
					{
						x_ray = x;
						y_ray = y;
						x_diff = float(x_grid - x);
						y_diff = float(y_grid - y);
						theta_diff = std::abs(std::fmod(theta - atan2(y_diff, x_diff) + pi, 2 * pi) - pi);
						int n = 4 * std::max(std::abs(sq(x_diff)), std::abs(sq(y_diff)));

						//ROS_INFO("x %f x_grid %f  y %f y_grid %f  x_diff/n %f  y_diff/n %f", x, x_grid, y, y_grid, x_diff / n, y_diff / n);
						for (int i_ray = 0; i_ray < n; i_ray++)
						{
							x_ray += x_diff / n;
							y_ray += y_diff / n;
							//ROS_INFO("Ray:  x %f y %f  wall_occ %f", x_ray, y_ray, wall_grid[sq(x_ray)][sq(y_ray)]);
							if (wall_grid[sq(x_ray)][sq(y_ray)] == 1.0)
							{
								add_exploration_cell = false;
								break;
							}
						}
					}
					if (add_exploration_cell)
					{
						//if (j < 1.0 || j > j_max - 1.5 || i < i_shift + 1.0 || i > i_max - 1.5)
						if (j > j_max - 1.5 || i < i_shift + 1.0 || i > i_max - 1.5)
						{

							int number_unexplored = 0;

							for (int i_frontier = sq(x_grid) - frontier_window_size; i_frontier < sq(x_grid) + frontier_window_size; i_frontier++)
							{
								for (int j_frontier = sq(y_grid) - frontier_window_size; j_frontier < sq(y_grid) + frontier_window_size; j_frontier++)
								{
									if (exploration_grid.at<float>(i_frontier, j_frontier) == 0.0)
									{
										number_unexplored++;
									}
								}
							}

							float cost = occupancy_grid.at<float>(sq(x_grid), sq(y_grid));
							//if (grid[sq(x_grid)][sq(y_grid)] < 1.0)
							if (cost < .6)
							{
								frontier_ptr frontier_node = std::make_shared<Frontier>(x_grid, y_grid, cost, theta_diff, frontier_distance, number_unexplored);
								frontier_nodes.push_back(frontier_node);
								all_frontiers_nodes.push_back(frontier_node);
								exploration_grid.at<float>(sq(x_grid), sq(y_grid)) = -1.0;
							}
						}
						else
							exploration_grid.at<float>(sq(x_grid), sq(y_grid)) = 1.0;
					}
				}
			}
		}

		grid_matrix_msg = publishExplorationGrid();

		geometry_msgs::Twist frontier_destination_pose;

		for (int i = 0; i < 10; i++)
			exploration_pub.publish(grid_matrix_msg);

		frontier_ptr frontier_destination_node;
		if (get_frontier)
		{
			std::vector<frontier_ptr>::iterator min_cost_iterator;

			ROS_INFO("Frontier size %d ", (int)frontier_nodes.size());
			ROS_INFO("All frontiers size %d ", (int)all_frontiers_nodes.size());
			if (false && !frontier_nodes.empty())
			{
				min_cost_iterator = std::min_element(frontier_nodes.begin(), frontier_nodes.end(), [](const frontier_ptr a, const frontier_ptr b) {
					return a->getCost() < b->getCost();
				});
				frontier_destination_node = frontier_nodes[std::distance(frontier_nodes.begin(), min_cost_iterator)];
			}
			else if (!all_frontiers_nodes.empty())
			{
				min_cost_iterator = std::min_element(all_frontiers_nodes.begin(), all_frontiers_nodes.end(), [](const frontier_ptr a, const frontier_ptr b) {
					return a->getCost() < b->getCost();
				});
				frontier_destination_node = all_frontiers_nodes[std::distance(all_frontiers_nodes.begin(), min_cost_iterator)];
				ROS_INFO(" ");
				ROS_INFO("Costs: %f occ %f, dist %f, dist_cost %f.  theta_diff %f  exp_cost %f", frontier_destination_node->getCost(), frontier_destination_node->occupancy_cost, frontier_destination_node->frontier_distance, frontier_destination_node->getDistanceCost(frontier_destination_node->frontier_distance, window_height), frontier_destination_node->theta_diff / (8 * pi), frontier_destination_node->getExplorationGainCost());
				ROS_INFO(" ");
			}
			else
			{
				ROS_INFO("Everything explored!");
				res.success = true;
				return frontier_destination_pose;
			}

			frontier_destination_pose.linear.x = frontier_destination_node->x;
			frontier_destination_pose.linear.y = frontier_destination_node->y;
		}
		else
			return frontier_destination_pose;

		return frontier_destination_pose;
	}

	bool withinMap(float x_grid, float y_grid)
	{
		return !(sq(x_grid) < 0) && !(sq(x_grid) >= num_grid_squares_x) && !(sq(y_grid) < 0) && !(sq(y_grid) >= num_grid_squares_y);
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
		wall_grid.resize(num_grid_squares_x);

		for (int i = 0; i < num_grid_squares_x; ++i)
		{
			grid[i].resize(num_grid_squares_y);
			wall_grid[i].resize(num_grid_squares_y);
		}

		updateBasicGrid();
		updateBasicWallGrid();
	}

	float distance_points(int x, int y, int a, int b)
	{
		float x_diff = x - a;
		float y_diff = y - b;
		return std::sqrt(x_diff * x_diff + y_diff * y_diff);
	}

	float distance(float x, float y, float x_to, float y_to)
	{
		float x_diff = x - x_to;
		float y_diff = y - y_to;
		return std::sqrt(x_diff * x_diff + y_diff * y_diff);

		if (distance_grid_init)
		{
			return float(distance_grid.at<int>(sq(x), sq(y))) * grid_square_size;
		}
		else
		{
			distance_grid = cv::Mat::zeros(num_grid_squares_x, num_grid_squares_y, CV_32SC1);

			if (!setDistance(sq(x_to), sq(y_to), 0))
			{
				ROS_WARN("No distance grid was generated for x:%f, y:%f", x_to, y_to);
				return 0;
			}
			else
			{
				distance_grid_init = true;

				//distance_matrix_msg = publishDistanceGrid();

				//for (int i = 0; i < 10; i++)
				//	distance_pub.publish(distance_matrix_msg);

				ROS_INFO("Distance %f", float(distance_grid.at<int>(sq(x), sq(y))) * grid_square_size);
				return float(distance_grid.at<int>(sq(x), sq(y))) * grid_square_size;
			}
			/*
		else
		{
			float x_diff = x - a;
			float y_diff = y - b;
			return std::sqrt(x_diff * x_diff + y_diff * y_diff);
		}
		*/
		}
	}

	bool setDistance(int x, int y, int dist)
	{
		if (x >= 0 && y >= 0 && x < num_grid_squares_x && y < num_grid_squares_y)
		{
			if (wall_grid[x][y] >= 1)
			{
				return false;
			}
			else
			{
				if (distance_grid.at<int>(x, y) > dist || distance_grid.at<int>(x, y) == 0)
				{

					distance_grid.at<int>(x, y) = dist;

					int x_pos = x + 1;
					int y_pos = y + 1;
					int x_neg = x - 1;
					int y_neg = y - 1;

					if (x_pos < num_grid_squares_x)
					{
						setDistance(x_pos, y, dist + 1);
					}
					if (x_neg >= 0)
					{
						setDistance(x_neg, y, dist + 1);
					}
					if (y_pos < num_grid_squares_y)
					{
						setDistance(x, y_pos, dist + 1);
					}
					if (y_neg >= 0)
					{
						setDistance(x, y_neg, dist + 1);
					}
				}
				return true;
			}
		}
		else
		{
			ROS_WARN("Outside of bounds, x:%d, y:%d, numsqx:%d, numsqy:%d", x, y, num_grid_squares_x, num_grid_squares_y);
			return false;
		}
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

	void updateBasicWallGrid()
	{
		ROS_DEBUG("Updating wall occupancy grid");

		// Put every point in the grid
		for (int i = 0; i < X_wall_coordinates.size(); ++i)
		{
			// Get the square that the point is located in
			int square_x = sq(X_wall_coordinates[i]);
			int square_y = sq(Y_wall_coordinates[i]);

			int x_low = square_x - num_wall_thickness_squares;
			int y_low = square_y - num_wall_thickness_squares;

			int x_high = square_x + num_wall_thickness_squares;
			int y_high = square_y + num_wall_thickness_squares;

			// Setting 1.0 for all the walls (enlarged by the min distance)
			for (int i = x_low; i <= x_high; ++i)
			{
				for (int j = y_low; j <= y_high; ++j)
				{
					if (!(i < 0) && !(i >= num_grid_squares_x) && !(j < 0) && !(j >= num_grid_squares_y))
					{
						// This keeps the wall expansion radial
						if (distance_points(square_x, square_y, i, j) <= (num_wall_thickness_squares))
						{
							wall_grid[i][j] = 1.0;
						}
					}
				}
			}
		}
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
				if (grid[i][j] >= 1)
				{
					normalized_grid_ones.at<float>(i, j) = 1;
				}
			}
		}

		ROS_INFO("Gaussed grid ready");
		return normalized_grid_ones;
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
	/*
	*/
	robo7_msgs::grid_matrix publishWallGrid()
	{
		robo7_msgs::grid_row grid_row_msg;
		robo7_msgs::grid_matrix grid_matrix_msg;

		std::vector<float> grid_row;

		for (int i = 0; i < wall_grid.size(); i++)
		{
			grid_row.clear();
			for (int j = 0; j < wall_grid[i].size(); j++)
			{
				grid_row.push_back((float)wall_grid[i][j]);
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
	float min_distance, wall_thickness;
	int num_min_distance_squares, num_wall_thickness_squares;
	float grid_square_size;
	int smoothing_kernel_size;
	int smoothing_kernel_sd;
	int num_grid_squares_x;
	int num_grid_squares_y;
	std::vector<float> X_wall_coordinates;
	std::vector<float> Y_wall_coordinates;
	cv::Mat occupancy_grid;
	cv::Mat basic_grid;
	cv::Mat exploration_grid;
	cv::Mat distance_grid;
	float current_x_to, current_y_to;
	bool occupancy_grid_init, exploration_grid_init, distance_grid_init;
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
