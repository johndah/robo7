#include <ros/ros.h>

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

//The services
#include <robo7_srvs/scanCoord.h>

#include <stdlib.h>
#include <string>
#include <vector>

float control_frequency = 10.0;


class MapMaintenance
{
public:
	ros::NodeHandle n;
	//Subscribers
	ros::Subscriber the_robot_pose_sub;
	ros::Subscriber state_activation_sub;
	ros::Subscriber wall_XY_sub;
	//Services client
	ros::ServiceClient scan_to_coord_srv;
	//Publishers
  ros::Publisher occupancy_grid_pub;

	MapMaintenance()
	{
		//Parameters input
		n.param<float>("/mapping/distance_between_two_measures", dist_threshold, 0.10);
		n.param<float>("/mapping/cell_size", cell_size, 0.10);
		n.param<float>("/mapping/free_space_around_a_cell", free_threshold, 0.10);
		n.param<bool>("/mapping/use_mapping_algorithm", use_mapping, false);
		n.param<bool>("/mapping/use_ransac", use_ransac, false);

		//Initialize state
		state_activated.mapping = false;
		condition_respected = true;
		occupied = 2;
		points_received = false;
		occupancy_initialized = false;

		//Subscribers
		the_robot_pose_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &MapMaintenance::robot_pose_callBack, this);
		state_activation_sub = n.subscribe("/robot_state/activation_states", 1, &MapMaintenance::state_callBack, this);
		wall_XY_sub = n.subscribe("/ras_maze/maze_map/walls_coord_for_icp", 1, &MapMaintenance::walls_callBack, this);

		//Service Clients
		scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");

		//Publishers
		occupancy_grid_pub = n.advertise<robo7_msgs::mapping_grid>("/localization/mapping/the_occupancy_grid", 1);
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

	void updateMap()
	{
		//Initialize the occupancy grid
		if(points_received&&!occupancy_initialized)
		{
			ROS_INFO("Point received");
			ROS_INFO("%d", the_wall_points.number);
			occupancy_initialisation();

			occupancy_initialized = true;
			ROS_INFO("Treated");
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
			update_the_occupancy_grid_with_lidar();

			//Save when/where was the last update
			previous_update_pose = the_robot_pose;

			//Look at the batteries
			update_the_occupancy_grid_with_battery();

			//Check if the map changed because of the lidar.
			if(new_change)
			{
				//Publish the occupancy grid
				occupancy_grid_pub.publish(the_occupancy_grid);

				//Then turn back the change value to false
				new_change = false;
			}

			//Turn back the condition to false
			condition_respected = false;
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

	//The occupancy grid msg
	robo7_msgs::mapping_grid the_occupancy_grid;

	//The discretized map
	robo7_msgs::wallPoint discretized_map;
	robo7_msgs::cornerList discretized_map_msg;
	robo7_msgs::cornerList the_wall_points;
	robo7_msgs::wallPoint map_lidar_scan;

	//Conditions triggers
	bool condition_respected, occupancy_initialized, points_received, new_change;
	float dist_threshold; float cell_size;
	float occupied, unoccupied, unknown;
	bool use_mapping, use_ransac;
	float x_max, x_min, y_max, y_min;
	float free_threshold;

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

	void update_the_occupancy_grid_with_lidar()
	{
		for(int i=0; i < map_lidar_scan.number; i++)
		{
			std::vector<int> the_cell = corresponding_cell(map_lidar_scan.the_points[i].x, map_lidar_scan.the_points[i].y);
			if((the_cell[0] > 0)&&(the_cell[0] < the_occupancy_grid.occupancy_grid.nb_rows)
					&&(the_cell[1] > 0)&&(the_cell[1] < the_occupancy_grid.occupancy_grid.nb_cols))
					{
						if(check_free_around_it(the_cell[0], the_cell[1]))
						{
							the_occupancy_grid.occupancy_grid.rows[the_cell[0]].cols[the_cell[1]] = occupied;
							new_change = true;
						}
					}
		}
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
				if((local_cell[0] > 0)&&(local_cell[0] < the_occupancy_grid.occupancy_grid.nb_rows)
						&&(local_cell[1] > 0)&&(local_cell[1] < the_occupancy_grid.occupancy_grid.nb_cols))
						{
							if(the_occupancy_grid.occupancy_grid.rows[local_cell[0]].cols[local_cell[1]] >= 1)
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
		the_occupancy_grid.top_left_corner.x = x_min;
		the_occupancy_grid.top_left_corner.y = y_min;
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
