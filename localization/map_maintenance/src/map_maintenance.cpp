#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <Eigen/Geometry>

//The messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_msgs/the_robot_position.h>
#include <robo7_msgs/activation_states.h>
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/mapping_grid.h>
#include <robo7_msgs/matrix.h>
#include <robo7_msgs/matrix_row.h>
#include <robo7_msgs/detectedObstacle.h>
#include <robo7_msgs/allObstacles.h>

//The services
#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/UpdateOccupancyGridFiltered.h>



float control_frequency = 10.0;


class MapMaintenance
{
public:
	ros::NodeHandle n;
	//Subscribers
	ros::Subscriber the_robot_pose_sub;
	ros::Subscriber state_activation_sub;
	ros::Subscriber wall_XY_sub;
	ros::Subscriber obstacle_sub;
	//Services client
	ros::ServiceClient scan_to_coord_srv;
	ros::ServiceClient update_occupancy_grid_srv;
	//Publishers
  ros::Publisher occupancy_grid_pub;
	ros::Publisher new_point_pub;
	ros::Publisher obstacle_pub;

	MapMaintenance()
	{
		//Parameters input
		n.param<float>("/map_maintenance/distance_between_two_measures", dist_threshold, 0.10);
		n.param<float>("/map_maintenance/cell_size", cell_size, 0.10);
		n.param<float>("/map_maintenance/free_space_around_a_cell", free_threshold, 0.10);
		n.param<float>("/map_maintenance/free_space_around_an_obstacle", obstacle_threshold, 0.20);
		n.param<float>("/map_maintenance/obstacle_width", obstacle_width, 0.10);
		n.param<float>("/map_maintenance/lidar_wall_detection_distance", lidar_distance_thres, 0.50);
		n.param<bool>("/map_maintenance/use_mapping_algorithm", use_mapping, false);
		n.param<bool>("/map_maintenance/use_ransac", use_ransac, false);

		//Initialize state
		state_activated.mapping = false;
		condition_respected = true;
		occupied = 2;
		obstacle = 3;
		points_received = false;
		occupancy_initialized = false;
		obstacle_vect.clear();
		all_obstacles_msg.obstacle_size = obstacle_width;

		//Subscribers
		the_robot_pose_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &MapMaintenance::robot_pose_callBack, this);
		state_activation_sub = n.subscribe("/robot_state/activation_states", 1, &MapMaintenance::state_callBack, this);
		wall_XY_sub = n.subscribe("/ras_maze/maze_map/walls_coord_for_icp", 1, &MapMaintenance::walls_callBack, this);
		obstacle_sub = n.subscribe("/vision/obstacle", 1, &MapMaintenance::obstacle_callBack, this);

		//Service Clients
		scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
		update_occupancy_grid_srv = n.serviceClient<robo7_srvs::UpdateOccupancyGridFiltered>("/occupancy_grid/update_occupancy_grid");

		//Publishers
		occupancy_grid_pub = n.advertise<robo7_msgs::mapping_grid>("/localization/mapping/the_occupancy_grid", 1);
		new_point_pub = n.advertise<robo7_msgs::wallPoint>("/localization/mapping/the_new_points", 1);
		obstacle_pub = n.advertise<robo7_msgs::allObstacles>("/localization/mapping/the_obstacles", 1);
	}

	void walls_callBack(const robo7_msgs::cornerList::ConstPtr &msg)
  {
		if(msg->number > 0)
		{
			the_wall_points = *msg;
			points_received = true;
		}
  }

	void robot_pose_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;
  }

	void state_callBack(const robo7_msgs::activation_states::ConstPtr &msg)
  {
		if(msg->header.seq == 1)
		{
			state_activated = *msg;
		}
  }

	void obstacle_callBack(const robo7_msgs::detectedObstacle::ConstPtr &msg)
	{
		obstacle_msg = *msg;
	}

	void updateMap()
	{
		//Initialize the occupancy grid
		if(points_received&&!occupancy_initialized&&state_activated.mapping)
		{
			occupancy_initialisation();

			occupancy_initialized = true;
		}

		//If the condition is respected
		if(condition_respected&&state_activated.mapping&&occupancy_initialized)
		{
			ROS_INFO("New update");
			//First you call back the lidar scan in map frame
			robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = the_robot_pose.position;
      req1.lidar_scan = the_robot_pose.the_lidar_scan;
      scan_to_coord_srv.call(req1, res1);
			map_lidar_scan = res1.the_lidar_points;

			//Fill up the previously undetected walls in the occupancy grid
			update_the_occupancy_grid_with_lidar( the_robot_pose.position );

			//Save when/where was the last update
			previous_update_pose = the_robot_pose;

			//Turn back the condition to false
			condition_respected = false;
		}

		//Look at the batteries
		update_the_occupancy_grid_with_battery();

		//Check if the map changed because of the lidar.
		if(new_change)
		{
			//Publish the occupancy grid
			occupancy_grid_pub.publish(the_occupancy_grid);

			//Update configuration space
			callUpdateService();

			//Then turn back the change value to false
			new_change = false;
		}

		//Definition pf the condition
		if(distance_between(previous_update_pose, the_robot_pose)&&use_mapping)
		{
			condition_respected = true;
		}
	}

private:
	//Subsribers values
	robo7_msgs::the_robot_position the_robot_pose;
	robo7_msgs::the_robot_position previous_update_pose;
	robo7_msgs::detectedObstacle obstacle_msg;

	//The occupancy grid msg
	robo7_msgs::mapping_grid the_occupancy_grid;
	std::vector<geometry_msgs::Vector3> obstacle_vect;
	robo7_msgs::allObstacles all_obstacles_msg;
	robo7_msgs::wallPoint the_new_points_msg;

	//The discretized map
	robo7_msgs::wallPoint discretized_map;
	robo7_msgs::cornerList discretized_map_msg;
	robo7_msgs::cornerList the_wall_points;
	robo7_msgs::wallPoint map_lidar_scan;

	//Conditions triggers
	bool condition_respected, occupancy_initialized, points_received, new_change;
	float dist_threshold; float cell_size;
	float occupied, unoccupied, unknown, obstacle;
	bool use_mapping, use_ransac;
	float x_max, x_min, y_max, y_min;
	float free_threshold, obstacle_threshold, obstacle_width, lidar_distance_thres;

	//State of the robot
	robo7_msgs::activation_states state_activated;

	bool distance_between(robo7_msgs::the_robot_position pose1, robo7_msgs::the_robot_position pose2)
	{
		float x1 = pose1.position.linear.x;
		float y1 = pose1.position.linear.y;
		float x2 = pose2.position.linear.x;
		float y2 = pose2.position.linear.y;

		if(sqrt( pow(x1-x2,2) + pow(y1-y2,2) > dist_threshold))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void update_the_occupancy_grid_with_lidar( geometry_msgs::Twist robot_pose )
	{
		for(int i=0; i < map_lidar_scan.number; i++)
		{
			if(distance_robot_point( robot_pose , map_lidar_scan.the_points[i] ) < lidar_distance_thres )
			{
				std::vector<int> the_cell = corresponding_cell(map_lidar_scan.the_points[i].x, map_lidar_scan.the_points[i].y);
				if((the_cell[0] > 0)&&(the_cell[0] < the_occupancy_grid.occupancy_grid.nb_rows)
						&&(the_cell[1] > 0)&&(the_cell[1] < the_occupancy_grid.occupancy_grid.nb_cols))
						{
							if(check_free_around_it(the_cell[0], the_cell[1]))
							{
								the_occupancy_grid.occupancy_grid.rows[the_cell[0]].cols[the_cell[1]] = occupied;
								new_change = true;
								the_new_points_msg.number++;
								the_new_points_msg.the_points.push_back(map_lidar_scan.the_points[i]);
							}
						}
			}
		}
		if(new_change)
		{
			new_point_pub.publish( the_new_points_msg );
		}
	}

	float distance_robot_point( geometry_msgs::Twist robot_pose , geometry_msgs::Vector3 one_lidar_point )
	{
		float x1 = robot_pose.linear.x;
		float y1 = robot_pose.linear.y;
		float x2 = one_lidar_point.x;
		float y2 = one_lidar_point.y;

		return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
	}

	std::vector<int> corresponding_cell(float x, float y)
	{
		std::vector<int> cell(2,0);
		cell[0] = (int)((x - the_occupancy_grid.top_left_corner.x)/the_occupancy_grid.cell_size + the_occupancy_grid.cell_size/2);
		cell[1] = (int)((y - the_occupancy_grid.top_left_corner.y)/the_occupancy_grid.cell_size + the_occupancy_grid.cell_size/2);

		return cell;
	}

	void update_the_occupancy_grid_with_battery()
	{
		if(obstacle_msg.flag.data)
		{
			geometry_msgs::Vector3 obstacle_position = from_robot_to_map_frame(obstacle_msg);
			// ROS_INFO("The obstacle map position : (x, y) = (%lf, %lf)", obstacle_position.x, obstacle_position.y);
			std::vector<int> the_obstacle_cell = corresponding_cell(obstacle_position.x, obstacle_position.y);
			if(check_free_around_it(the_obstacle_cell[0], the_obstacle_cell[1]))
			{
				// ROS_INFO("The cell around are free");
				bool already_in = false;
				for(int i=0; i < static_cast<int>(obstacle_vect.size()); i++)
				{
					geometry_msgs::Vector3 previously_detected_obstacle = obstacle_vect[i];
					//If the obstacle is too close of an existing obstacle, then don't add it
					if(check_distance_between_obstacles(obstacle_position, previously_detected_obstacle))
					{
						already_in = true;
					}
				}
				if(!already_in)
				{
					// ROS_INFO("New obstacle has been detected");
					//Update the grid with a battery
					update_grid_with_square( obstacle_position );

					//Add this battery to the obstacle vector
					obstacle_vect.push_back(obstacle_position);

					//The obstacle message
					all_obstacles_msg.number++;
					all_obstacles_msg.the_obstacles.push_back( obstacle_position );
					//Then publish it
					obstacle_pub.publish(all_obstacles_msg);

					//New update has been done
					new_change = true;
				}
			}

		}
	}

	geometry_msgs::Vector3 from_robot_to_map_frame(robo7_msgs::detectedObstacle anObstacle)
	{
		float robot_radius = 0.13;
		//Matrices definition
		Eigen::Vector3f obstacle_robot_position_vector;
		obstacle_robot_position_vector(0) = robot_radius + obstacle_width/2 + anObstacle.dist;
    obstacle_robot_position_vector(1) = -(anObstacle.x2 + anObstacle.x1)/2;
    obstacle_robot_position_vector(2) = 0;

		Eigen::Matrix3f rotation_matrix_map = Eigen::Matrix3f::Zero(3,3);
    float robot_angle = the_robot_pose.position.angular.z;
    rotation_matrix_map(0,0) = cos(robot_angle);
    rotation_matrix_map(0,1) = -sin(robot_angle);
    rotation_matrix_map(1,0) = sin(robot_angle);
    rotation_matrix_map(1,1) = cos(robot_angle);
    rotation_matrix_map(2,2) = 1;

		Eigen::Vector3f translation_vector_map;
    translation_vector_map(0) = the_robot_pose.position.linear.x;
    translation_vector_map(1) = the_robot_pose.position.linear.y;
    translation_vector_map(2) = the_robot_pose.position.linear.z;

		Eigen::Vector3f obstacle_map_position_vector;
		obstacle_map_position_vector = rotation_matrix_map * obstacle_robot_position_vector + translation_vector_map;

		geometry_msgs::Vector3 obstacle_in_map;
		obstacle_in_map.x = obstacle_map_position_vector(0);
		obstacle_in_map.y = obstacle_map_position_vector(1);
		obstacle_in_map.z = obstacle_map_position_vector(2);

		return obstacle_in_map;
	}

	bool check_distance_between_obstacles(geometry_msgs::Vector3 obstacle_1, geometry_msgs::Vector3 obstacle_2)
	{
		if(sqrt(pow(obstacle_1.x-obstacle_2.x,2) + pow(obstacle_1.y-obstacle_2.y,2)) < obstacle_threshold)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void update_grid_with_square(geometry_msgs::Vector3 anObstacle)
	{
		int cell_around = (int)(obstacle_width/cell_size);
		std::vector<int> the_obstacle_cell = corresponding_cell(anObstacle.x, anObstacle.y);
		for(int i=-cell_around+1; i<cell_around; i++)
		{
			for(int j=-cell_around+1; j<cell_around; j++)
			{
				std::vector<int> local_cell(2.0);
				local_cell[0] = the_obstacle_cell[0] + i;
				local_cell[1] = the_obstacle_cell[1] + j;
				if((local_cell[0] >= 0)&&(local_cell[0] < the_occupancy_grid.occupancy_grid.nb_rows)
						&&(local_cell[1] >= 0)&&(local_cell[1] < the_occupancy_grid.occupancy_grid.nb_cols))
						{
							the_occupancy_grid.occupancy_grid.rows[local_cell[0]].cols[local_cell[1]] = obstacle;
						}
			}
		}
	}

	bool check_free_around_it(int i_ind, int j_ind)
	{
		int cell_around = (int)(free_threshold/cell_size);
		for(int i=-cell_around+1; i<cell_around; i++)
		{
			for(int j=-cell_around+1; j<cell_around; j++)
			{
				std::vector<int> local_cell(2,0);
				local_cell[0] = i_ind + i;
				local_cell[1] = j_ind + j;
				if((local_cell[0] >= 0)&&(local_cell[0] < the_occupancy_grid.occupancy_grid.nb_rows)
						&&(local_cell[1] >= 0)&&(local_cell[1] < the_occupancy_grid.occupancy_grid.nb_cols))
						{
							if(the_occupancy_grid.occupancy_grid.rows[local_cell[0]].cols[local_cell[1]] == occupied)
							{
								return false;
							}
						}
			}
		}

		return true;
	}

	void occupancy_initialisation()
	{
		the_occupancy_grid.cell_size = cell_size;
		//Extract the coordinates out of the XY_coordinates of the walls
		find_min_max();
		//Defining the window size
		the_occupancy_grid.window_width = (int)((x_max - x_min)/cell_size + 1) * cell_size;
		the_occupancy_grid.window_height = (int)((y_max - y_min)/cell_size + 1) * cell_size;
		//Top left corner
		the_occupancy_grid.top_left_corner.x = x_min - cell_size/2;
		the_occupancy_grid.top_left_corner.y = y_min - cell_size/2;
		//Define the occupancy matrix
		robo7_msgs::matrix the_matrix;
		the_matrix.nb_rows = (int)(the_occupancy_grid.window_width / the_occupancy_grid.cell_size);
		the_matrix.nb_cols = (int)(the_occupancy_grid.window_height / the_occupancy_grid.cell_size);
		robo7_msgs::matrix_row one_row;
		for(int i=0; i<the_matrix.nb_cols; i++)
		{
			one_row.cols.push_back(0);
		}
		for(int i=0; i < the_matrix.nb_rows; i++)
		{
			the_matrix.rows.push_back(one_row);
		}
		for(int i=0; i<the_wall_points.number; i++)
		{
			std::vector<int> the_cell = corresponding_cell(the_wall_points.corners[i].x, the_wall_points.corners[i].y);
			the_matrix.rows[the_cell[0]].cols[the_cell[1]] = occupied;
		}
		the_occupancy_grid.occupancy_grid = the_matrix;
	}

	void find_min_max()
	{
		if(the_wall_points.number > 0)
		{
			//Initialization
			x_max = the_wall_points.corners[0].x;
			x_min = the_wall_points.corners[0].x;
			y_max = the_wall_points.corners[0].y;
			y_min = the_wall_points.corners[0].y;

			//Find the minimum/maximum
			for(int i=0; i<the_wall_points.number; i++)
			{
				if(the_wall_points.corners[i].x > x_max){x_max = the_wall_points.corners[i].x;}
				if(the_wall_points.corners[i].x < x_min){x_min = the_wall_points.corners[i].x;}
				if(the_wall_points.corners[i].y > y_max){y_max = the_wall_points.corners[i].y;}
				if(the_wall_points.corners[i].y < y_min){y_min = the_wall_points.corners[i].y;}
			}
		}
	}

	void callUpdateService()
	{
		robo7_srvs::UpdateOccupancyGridFiltered::Request req2;
		robo7_srvs::UpdateOccupancyGridFiltered::Response res2;
		req2.new_points = the_new_points_msg;
		req2.the_obstacles = all_obstacles_msg;
		update_occupancy_grid_srv.call(req2, res2);
	}
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MapMaintenance");

    MapMaintenance MapMaintenance_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("MapMaintenance is turning");

    while(MapMaintenance_.n.ok())
    {
        MapMaintenance_.updateMap();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
