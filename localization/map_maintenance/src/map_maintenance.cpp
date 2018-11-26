#include <ros/ros.h>

//The messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_msgs/the_robot_position.h>
#include <robo7_msgs/activation_states.h>

//The services
#include <robo7_srvs/UpdateOccupancyGrid.h>
#include <robo7_srvs/UpdateDiscretizedMap.h>

#include <stdlib.h>
#include <string>
#include <vector>

float control_frequency = 10.0;


class Mapping
{
public:
	ros::NodeHandle n;
	//Subscribers
	ros::Subscriber the_robot_pose_sub;
	ros::Subscriber state_activation_sub;
	//Services client
	ros::ServiceClient update_occupancy_grid_srv;
	ros::ServiceClient update_discretized_map_srv;
	//Publishers
  ros::Publisher updated_discretized_map_pub;
	ros::Publisher activation_states_pub;

	Mapping()
	{
		//Parameters input
		n.param<float>("/mapping/distance_between_two_measures", dist_threshold, 0.10);
		n.param<float>("/mapping/cell_size", cell_size, 0.10);
		n.param<bool>("/mapping/use_mapping_algorithm", use_mapping, false);
		n.param<bool>("/mapping/use_ransac", use_ransac, false);

		//Initialize state
		state_activated.mapping = false;
		condition_respected = true;
		occupied = 2;
		initialize_occupancy_grid();

		//Subscribers
		the_robot_pose_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &Mapping::robot_pose_callBack, this);
		state_activation_sub = n.subscribe("/robot_state/activation_states", 1, &Mapping::state_callBack, this);

		//Service Clients
		update_occupancy_grid_srv = n.serviceClient<robo7_srvs::UpdateOccupancyGrid>("/localization/mapping/update_occupancy_grid");
		update_discretized_map_srv = n.serviceClient<robo7_srvs::UpdateDiscretizedMap>("/localization/mapping/update_discretized_map");

		//Publishers
		updated_discretized_map_pub = n.advertise<robo7_msgs::cornerList>("/localization/mapping/slam_map", 1);
		activation_states_pub = n.advertise<robo7_msgs::activation_states>("/robot_state/activation_states", 1);

		//Shitty published
		activation_states_pub.publish( state_activated );
		activation_states_pub.publish( state_activated );
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
		//If the condition is respected
		if(condition_respected&&state_activated.mapping)
		{
			ROS_INFO("New update");
			//First you call back the lidar scan in map frame
			

			//Fill up the previously undetected walls in the occupancy grid
			update_the_occupancy_grid_with_lidar();

			//Save when/where was the last update
			previous_update_pose = the_robot_pose;

			//Finally you publish the newly discretized map so that the robot can use'
			//for localizing itself

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

	//Conditions triggers
	bool condition_respected;
	float dist_threshold; float cell_size;
	float occupied, unoccupied, unknown;
	bool use_mapping, use_ransac;

	//State of the robot
	robo7_msgs::activation_states state_activated;

	void initialize_occupancy_grid()
	{
		//Initialize the matrice
		robo7_msgs::matrix initial_grid;
		robo7_msgs::matrix_row one_row;
    one_row.cols.clear();
    one_row.cols.push_back(0);
    initial_grid.nb_rows = 1;
    initial_grid.nb_cols = 1;
    initial_grid.rows.push_back(one_row);

		//Initialize the message
		the_occupancy_grid.header.seq = 0;
    the_occupancy_grid.header.stamp = ros::Time::now();
    the_occupancy_grid.window_width = cell_size;
    the_occupancy_grid.window_height = cell_size;
    the_occupancy_grid.cell_size = cell_size;
    the_occupancy_grid.top_left_corner.x = cell_size * (int)(the_robot_pose.position.linear.x/cell_size);
    the_occupancy_grid.top_left_corner.y = cell_size * (int)(the_robot_pose.position.linear.y/cell_size);
    the_occupancy_grid.occupancy_grid = initial_grid;
	}

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

	geometry_msgs::Vector3 corresponding_coordinates(int i, int j)
	{
		float x_value = the_occupancy_grid.top_left_corner.x + (j+1) * cell_size - cell_size/2;
		float y_value = the_occupancy_grid.top_left_corner.y - (i+1) * cell_size + cell_size/2;

		geometry_msgs::Vector3 wall_point;
		wall_point.x = x_value;
		wall_point.y = y_value;
		wall_point.z = 0;

		return wall_point;
	}


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mapping");

    Mapping Mapping_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("KMapping is turning");

    while(Mapping_.n.ok())
    {
        Mapping_.updateMap();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
