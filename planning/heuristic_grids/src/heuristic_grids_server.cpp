#include <algorithm>
#include <math.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/distanceTo.h"
#include "robo7_srvs/UpdateOccupancyGridFiltered.h"
#include "robo7_msgs/XY_coordinates.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <image_transport/image_transport.h>
#include "robo7_msgs/occupancy_matrix.h"
#include "robo7_msgs/occupancy_row.h"
#include "robo7_msgs/grid_matrix.h"
#include "robo7_msgs/grid_row.h"
#include "robo7_msgs/wallPoint.h"
#include "robo7_msgs/allObstacles.h"

typedef std::vector<double> Array;
typedef std::vector<Array> Matrix;

class HeuristicGridsServer
{
  public:
	ros::NodeHandle n;
	ros::Subscriber map_sub, new_point_sub, obstacle_sub, current_occupancy_sub;
	ros::Publisher occupancy_pub, distance_pub;
	robo7_msgs::occupancy_matrix occupancy_matrix_msg, distance_matrix_msg;
	ros::ServiceServer is_occupied_service;
	ros::ServiceServer distance_to_service;
  ros::ServiceServer update_occupancy_service;
	Matrix grid;

	HeuristicGridsServer()
	{
		// Parameters
		n.param<double>("/heuristic_grids_server/grid_square_size", grid_square_size, 0.02);
		n.param<double>("/heuristic_grids_server/min_distance", min_distance, 0.13);
		n.param<int>("/heuristic_grids_server/smoothing_kernel_size", smoothing_kernel_size, 15);
		n.param<int>("/heuristic_grids_server/smoothing_kernel_sd", smoothing_kernel_sd, 3);

    //The different subscribes
		map_sub = n.subscribe("/own_map/wall_coordinates", 1, &HeuristicGridsServer::mapCallback, this);
    new_point_sub = n.subscribe("/localization/mapping/the_new_points", 1, &HeuristicGridsServer::newpointCallback, this);
    obstacle_sub = n.subscribe("/localization/mapping/the_obstacles", 1, &HeuristicGridsServer::obstacleCallback, this);
    current_occupancy_sub = n.subscribe("/localization/mapping/the_occupancy_grid", 1, &HeuristicGridsServer::occupancyCallback, this);

    is_occupied_service = n.advertiseService("/occupancy_grid/is_occupied", &HeuristicGridsServer::occupancyGridRequest, this);
		distance_to_service = n.advertiseService("/distance_grid/distance", &HeuristicGridsServer::distanceGridRequest, this);

    update_occupancy_service = n.advertiseService("/occupancy_grid/update_occupancy_grid", &HeuristicGridsServer::occupancyGridUpdateRequest, this);

		occupancy_pub = n.advertise<robo7_msgs::occupancy_matrix>("/heuristic_grids_server/occupancy_matrix", 1);
		distance_pub = n.advertise<robo7_msgs::occupancy_matrix>("/heuristic_grids_server/distance_matrix", 1);

		num_min_distance_squares = ceil(min_distance / grid_square_size);

		if (smoothing_kernel_size % 2 == 0)
		{
			smoothing_kernel_size += 1;
			ROS_WARN("Entered kernel size is even number, changing to: %d", smoothing_kernel_size);
		}
	}

  void mapCallback(const robo7_msgs::XY_coordinates::ConstPtr &msg)
	{
		X_wall_coordinates = msg->X_coordinates;
		Y_wall_coordinates = msg->Y_coordinates;
	}

  void newpointCallback(const robo7_msgs::wallPoint::ConstPtr &msg)
	{
		new_point_list_msg = *msg;
	}

  void obstacleCallback(const robo7_msgs::allObstacles::ConstPtr &msg)
	{
		the_obstacles_msg = *msg;
	}

  void occupancyCallback(const robo7_msgs::mapping_grid::ConstPtr &msg)
  {
    the_occupancy_grid_msg = *msg;
  }

	int sq(double coord)
	{
		return floor(coord / grid_square_size);
	}

	bool distanceGridRequest(robo7_srvs::distanceTo::Request &req,
							 robo7_srvs::distanceTo::Response &res)
	{
		ROS_DEBUG("New grid distance request recieved");

		float x_from = req.x_from;
		float y_from = req.y_from;
		float x_to = req.x_to;
		float y_to = req.y_to;

		if (x_to != current_x_to || y_to != current_y_to)
		{
			distance_grid_init = false;
			current_x_to = x_to;
			current_y_to = y_to;
			ROS_INFO("New target!");
		}

		res.distance = getDistanceFromGrid(x_to, y_to, x_from, y_from);

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
			occupancy_matrix_msg = publishOccupancyGrid();

			for (int i = 0; i < 10; i++)
				occupancy_pub.publish(occupancy_matrix_msg);

			occupancy_grid_init = true;
		}

		return true;
	}

	void updateBasicGridSize()
	{
		ROS_DEBUG("Setting heuristic grids size");

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
    updateFilteredGrid( grid );
	}

	double distance_points(int x, int y, int a, int b)
	{
		double x_diff = x - a;
		double y_diff = y - b;
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
  }

  void updateFilteredGrid(Matrix grid_instant)
  {
		// Setting the filtered grid

		cv::Mat basic_grid(grid_instant.size(), grid_instant.at(0).size(), CV_64FC1);

		for (int i = 0; i < basic_grid.rows; ++i)
		{
			for (int j = 0; j < basic_grid.cols; ++j)
			{
				basic_grid.at<double>(i, j) = grid_instant[i][j];
			}
		}

		occupancy_grid = gaussFilter(basic_grid, smoothing_kernel_size, smoothing_kernel_sd, grid_instant);
	}

	cv::Mat gaussFilter(cv::Mat grid_in, int kernel_size, int sigma, Matrix grid_instant)
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
				if (grid_instant[i][j] >= 1)
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

	bool setDistance(int x, int y, int dist)
	{
		if (x >= 0 && y >= 0 && x < num_grid_squares_x && y < num_grid_squares_y)
		{

			if (grid[x][y] >= 1)
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

	int getDistanceFromGrid(double x_to, double y_to, double x_from, double y_from)
	{

		if (distance_grid_init)
		{
			return distance_grid.at<int>(sq(x_from), sq(y_from));
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
				// To display, uncomment bellow
				// cv::Mat normalized_dist_grid;
				// cv::normalize(distance_grid, normalized_dist_grid, 0, 255, cv::NORM_MINMAX, CV_8UC3);
				// namedWindow("Display window", cv::WINDOW_NORMAL );
				// cv::resizeWindow("Display window", 600,600);
				// imshow( "Display window", normalized_dist_grid );
				// cv::waitKey(0);
				// cvDestroyWindow("Display window");

				distance_grid_init = true;

				distance_matrix_msg = publishDistanceGrid();

				for (int i = 0; i < 10; i++)
					distance_pub.publish(distance_matrix_msg);

				return distance_grid.at<int>(sq(x_from), sq(y_from));
			}
		}
	}

	robo7_msgs::occupancy_matrix publishOccupancyGrid()
	{
		robo7_msgs::occupancy_row occupancy_row_msg;
		robo7_msgs::occupancy_matrix occupancy_matrix_msg;

		std::vector<float> occupancy_row;

		for (int i = 0; i < occupancy_grid.rows; i++)
		{
			occupancy_row.clear();
			for (int j = 0; j < occupancy_grid.cols; j++)
			{
				occupancy_row.push_back(occupancy_grid.at<float>(i, j));
			}
			occupancy_row_msg.occupancy_row = occupancy_row;
			occupancy_matrix_msg.occupancy_rows.push_back(occupancy_row_msg);
		}

		return occupancy_matrix_msg;
	}

	robo7_msgs::occupancy_matrix publishDistanceGrid()
	{
		robo7_msgs::occupancy_row distance_row_msg;
		robo7_msgs::occupancy_matrix distance_matrix_msg;

		std::vector<float> distance_row;

		for (int i = 0; i < distance_grid.rows; i++)
		{
			distance_row.clear();
			for (int j = 0; j < distance_grid.cols; j++)
			{
				distance_row.push_back(distance_grid.at<float>(i, j));
			}
			distance_row_msg.occupancy_row = distance_row;
			distance_matrix_msg.occupancy_rows.push_back(distance_row_msg);
		}

		return distance_matrix_msg;
	}

  //Here stands an updated version of the occupancy grid that is recomputed
  //everytime we need to do it (new detection)
  bool occupancyGridUpdateRequest(robo7_srvs::UpdateOccupancyGridFiltered::Request &req,
							  robo7_srvs::UpdateOccupancyGridFiltered::Response &res)
	{
    //Call back the variable
    // the_occupancy_grid_msg = req.current_occupancy_grid;

    // Set up sizes
    if(false)
    {
  		grid_mapping.resize(the_occupancy_grid_msg.occupancy_grid.nb_rows);
  		for (int i = 0; i < the_occupancy_grid_msg.occupancy_grid.nb_rows; ++i)
  			grid_mapping[i].resize(the_occupancy_grid_msg.occupancy_grid.nb_cols);

      for(int i=0; i<the_occupancy_grid_msg.occupancy_grid.nb_rows; i++)
      {
        for(int j=0; j<the_occupancy_grid_msg.occupancy_grid.nb_rows; j++)
        {
          grid_mapping[i][j] = the_occupancy_grid_msg.occupancy_grid.rows[i].cols[j];
        }
      }
    }
    else
    {
      grid_mapping = grid;
      int cell_around = (int)(min_distance/grid_square_size)+2;
      float dist = min_distance;
      //Add the new points
      for(int k=0; k < new_point_list_msg.number; k++)
      {
        float x_loc = new_point_list_msg.the_points[k].x;
        float y_loc = new_point_list_msg.the_points[k].y;
        fill_local_cells(x_loc, y_loc, dist, cell_around);
      }

      //Add the obstacles
      cell_around = (int)(min_distance/grid_square_size)+2;
      for(int k=0; k < the_obstacles_msg.number; k++)
      {
        float x_loc = the_obstacles_msg.the_obstacles[k].x;
        float y_loc = the_obstacles_msg.the_obstacles[k].y;
        float o_size = the_obstacles_msg.obstacle_size/2;
        for(float y_spec = y_loc - o_size; y_spec < y_loc + o_size; y_spec += grid_square_size)
        {
          float x_spec = x_loc - o_size;
          fill_local_cells(x_spec, y_spec, dist, cell_around);
          x_spec = x_loc + o_size;
          fill_local_cells(x_spec, y_spec, dist, cell_around);
        }
        for(float x_spec = x_loc - o_size; x_spec < x_loc + o_size; x_spec += grid_square_size)
        {
          float y_spec = y_loc - o_size;
          fill_local_cells(x_spec, y_spec, dist, cell_around);
          y_spec = y_loc + o_size;
          fill_local_cells(x_spec, y_spec, dist, cell_around);
        }
      }
    }

    updateFilteredGrid( grid_mapping );

    //then publish this new blured grid
    robo7_msgs::occupancy_matrix occupancy_matrix_msg;
    occupancy_matrix_msg = publishOccupancyGrid();

    occupancy_pub.publish( occupancy_matrix_msg );

    res.success = true;
		return true;
	}

  std::vector<int> find_index(float x, float y)
  {
    std::vector<int> the_local_cell(2,0);

    the_local_cell[0] = (int)(x/grid_square_size + grid_square_size/2);
    the_local_cell[1] = (int)(y/grid_square_size + grid_square_size/2);

    return the_local_cell;
  }

  void fill_local_cells(float x_pos, float y_pos, float dist, int cell_around)
  {
    std::vector<int> the_cell_index = find_index(x_pos, y_pos);
    for(int i=-cell_around+1; i<cell_around; i++)
    {
      for(int j=-cell_around+1; j<cell_around; j++)
      {
        int local_i = the_cell_index[0] + i;
        int local_j = the_cell_index[1] + j;
        if(distance_lower(x_pos, y_pos, local_i, local_j, dist))
        {
          if((local_i>0)&&(local_i < static_cast<int>(grid_mapping.size()))
              &&(local_j>0)&&(local_j < static_cast<int>(grid_mapping[0].size())))
              {
                grid_mapping[local_i][local_j] = 1.0;
              }
        }
      }
    }
  }

  bool distance_lower(float x1, float y1, int ind_i, int ind_j, float dist)
  {
    float x = (ind_i + 1/2) * grid_square_size;
    float y = (ind_j + 1/2) * grid_square_size;

    return (sqrt(pow(x-x1,2)+pow(y-y1,2)) < dist);
  }

  private:
	double min_distance;
	int num_min_distance_squares;
	double grid_square_size;
	int smoothing_kernel_size;
	int smoothing_kernel_sd;
	int num_grid_squares_x;
	int num_grid_squares_y;
	std::vector<float> X_wall_coordinates;
	std::vector<float> Y_wall_coordinates;
	cv::Mat basic_grid;
	cv::Mat occupancy_grid;
	cv::Mat distance_grid;
	float current_x_to, current_y_to;
	bool occupancy_grid_init, distance_grid_init;
  Matrix grid_mapping;
  robo7_msgs::wallPoint new_point_list_msg;
  robo7_msgs::allObstacles the_obstacles_msg;
  robo7_msgs::mapping_grid the_occupancy_grid_msg;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "heuristic_grids_server");

	HeuristicGridsServer heuristic_grids_server;

	ros::Rate loop_rate(10);

	ROS_INFO("Heuristic grids server running");

	heuristic_grids_server.updateBasicGridSize();

  robo7_msgs::occupancy_matrix occupancy_matrix_msg;
  robo7_msgs::occupancy_matrix distance_matrix_msg;
	while (ros::ok())
	{
		occupancy_matrix_msg = heuristic_grids_server.publishOccupancyGrid();
		distance_matrix_msg = heuristic_grids_server.publishDistanceGrid();

		heuristic_grids_server.occupancy_pub.publish(occupancy_matrix_msg);
		heuristic_grids_server.distance_pub.publish(distance_matrix_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

  ros::spin();

	return 0;
}
