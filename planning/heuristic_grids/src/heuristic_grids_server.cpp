#include <algorithm>
#include <math.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/distanceTo.h"
#include "robo7_msgs/XY_coordinates.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <image_transport/image_transport.h>
#include "robo7_msgs/occupancy_matrix.h"
#include "robo7_msgs/occupancy_row.h"

typedef std::vector<double> Array;
typedef std::vector<Array> Matrix;

class HeuristicGridsServer
{
  public:
	ros::NodeHandle n;
	ros::Subscriber map_sub;
  ros::Publisher occupancy_pub;
	ros::ServiceServer is_occupied_service;
	ros::ServiceServer distance_to_service;
	Matrix grid;

	HeuristicGridsServer()
	{
		// Parameters
		n.param<double>("/heuristic_grids_server/grid_square_size", grid_square_size, 0.02);
		n.param<double>("/heuristic_grids_server/min_distance", min_distance, 0.13);
		n.param<int>("/heuristic_grids_server/smoothing_kernel_size", smoothing_kernel_size, 15);
		n.param<int>("/heuristic_grids_server/smoothing_kernel_sd", smoothing_kernel_sd, 3);

		map_sub = n.subscribe("/own_map/wall_coordinates", 1, &HeuristicGridsServer::mapCallback, this);
		is_occupied_service = n.advertiseService("/occupancy_grid/is_occupied", &HeuristicGridsServer::occupancyGridRequest, this);
    distance_to_service = n.advertiseService("/distance_grid/distance", &HeuristicGridsServer::distanceGridRequest, this);

    occupancy_pub = n.advertise<robo7_msgs::occupancy_matrix>("occupancy_matrix", 1);

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

  int sq(double coord){
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
            if(distance_points(square_x, square_y, i, j) <= (num_min_distance_squares))
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
        basic_grid.at<double>(i, j) = grid[i][j];
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

    for (int i = 0; i < normalized_grid.rows; ++i){
      for (int j = 0; j < normalized_grid.cols; ++j){
        // if(unfiltered_grid[i][j] >= 1){
        if(grid[i][j] >= 1){
          normalized_grid_ones.at<float>(i, j) = 1 ;
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


  bool setDistance(int x, int y, int dist){
    if(x >= 0 && y >= 0 && x < num_grid_squares_x && y < num_grid_squares_y){

      if(grid[x][y] >= 1){
        return false;

      } else{
        if(distance_grid.at<int>(x, y) > dist || distance_grid.at<int>(x, y) == 0){

          distance_grid.at<int>(x, y) = dist;

          int x_pos = x + 1;
          int y_pos = y + 1;
          int x_neg = x - 1;
          int y_neg = y - 1;

          if (x_pos < num_grid_squares_x){
            setDistance(x_pos, y, dist+1);
          }
          if (x_neg >= 0){
            setDistance(x_neg, y, dist+1);
          }
          if (y_pos < num_grid_squares_y){
            setDistance(x, y_pos, dist+1);
          }
          if (y_neg >= 0){
            setDistance(x, y_neg, dist+1);
          }
        }
        return true;
      }
    } else{
      ROS_WARN("Outside of bounds, x:%d, y:%d, numsqx:%d, numsqy:%d", x, y, num_grid_squares_x, num_grid_squares_y);
      return false;
    }
  }


  int getDistanceFromGrid(double x_to, double y_to, double x_from, double y_from){
    distance_grid = cv::Mat::zeros(num_grid_squares_x, num_grid_squares_y, CV_32SC1);

    if(!setDistance(sq(x_to), sq(y_to), 0)){
      ROS_WARN("No distance grid was generated for x:%f, y:%f", x_to, y_to);
      return 0;
    } else{

      // To display, uncomment bellow
      // cv::Mat normalized_dist_grid;
      // cv::normalize(distance_grid, normalized_dist_grid, 0, 255, cv::NORM_MINMAX, CV_8UC3);
      // namedWindow("Display window", cv::WINDOW_NORMAL );
      // cv::resizeWindow("Display window", 600,600);
      // imshow( "Display window", normalized_dist_grid );
      // cv::waitKey(0);
      // cvDestroyWindow("Display window");

      return distance_grid.at<int>(sq(x_from), sq(y_from));
    }
  }


	void publishOccupancyGrid()
	{
		robo7_msgs::occupancy_row occupancy_row_msg;
		robo7_msgs::occupancy_matrix occupancy_matrix_msg;

		for (int i = 0; i < occupancy_grid.rows; ++i)
		{
			std::vector<float> occupancy_row(occupancy_grid.cols);
			for (int j = 0; j < occupancy_grid.cols; ++j)
			{
				occupancy_row.push_back(occupancy_grid.at<float>(i, j));
			}
			occupancy_row_msg.occupancy_row = occupancy_row;
			occupancy_matrix_msg.occupancy_rows.push_back(occupancy_row_msg);
		}

		for (int i = 0; i < 1000; i++)
		{
			occupancy_pub.publish(occupancy_matrix_msg);
		}
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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "heuristic_grids_server");

	HeuristicGridsServer heuristic_grids_server;

	ros::Rate loop_rate(10);

	ROS_INFO("Heuristic grids server running");

	heuristic_grids_server.updateBasicGridSize();

	heuristic_grids_server.publishOccupancyGrid();

	ros::spin();

	return 0;
}
