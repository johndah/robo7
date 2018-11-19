#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>

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



class Initialisation
{
public:
	ros::NodeHandle n;
	//Services client
	ros::ServiceClient update_occupancy_grid_srv;
	//Publishers
  ros::Publisher activation_states_pub;

	float time_before_initialisation;

	Initialisation()
	{
		//Parameters input
		n.param<bool>("/initialisation/use_mapping_mode", mapping_mode, false);
		n.param<float>("/initialisation/time_before_starting_everything", time_before_initialisation, 5.0);

		//Publishers
		activation_states_pub = n.advertise<robo7_msgs::activation_states>("/robot_state/activation_states", 1);
	}

	void first_publish()
	{
		//Robot actions state
		state_activated.header.seq = 0;
		state_activated.header.stamp = ros::Time::now();
		state_activated.follow_point = false;
		state_activated.localize_itself = false;
		state_activated.mapping = false;

		activation_states_pub.publish( state_activated );
	}

	void initialisation_sequence()
	{
		header_initialisation();
		if(!mapping_mode)
		{
			//Here should stand the initialisation for the non mapping mode
			//Otherwise it is the picking up sequence mode
			initialize_pickingup_states();
		}
		else
		{
			//Here stands the initialisation for the mapping mode

			//Start the mapping mode
			initialize_mapping_states();
		}

		//Publish the initial state of the robot
		activation_states_pub.publish( state_activated );
	}

private:
	//Initialisation published_msgs
	robo7_msgs::activation_states state_activated;

	//The mode we choose
	bool mapping_mode;

	void header_initialisation()
	{
		state_activated.header.seq = 1;
		state_activated.header.stamp = ros::Time::now();
	}

	void initialize_pickingup_states()
	{

		//Robot actions state
		state_activated.follow_point = false;
		state_activated.localize_itself = true;
		state_activated.mapping = false;
	}

	void initialize_mapping_states()
	{
		//Robot state

		//Robot actions state
		state_activated.follow_point = false;
		state_activated.localize_itself = false;
		state_activated.mapping = true;
	}

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Initialisation service");

	Initialisation Initialisation_;

	ros::Duration(1.0).sleep(); // sleep for half a second
	Initialisation_.first_publish();

	ros::Rate loop_rate(100);

	ros::Duration(Initialisation_.time_before_initialisation).sleep(); // sleep for half a second

	ros::spinOnce();

	Initialisation_.initialisation_sequence();

	ROS_INFO("Initialisation state of the robot done");

	return 0;
}
