#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>

//The messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/wallPoint.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_msgs/allObstacles.h>
#include <robo7_msgs/the_robot_position.h>
#include <robo7_msgs/activation_states.h>


//The services
#include <robo7_srvs/UpdateOccupancyGrid.h>
#include <robo7_srvs/UpdateDiscretizedMap.h>
#include <robo7_srvs/UpdateOccupancyGridFiltered.h>



class Initialisation
{
public:
	ros::NodeHandle n;

	//Publishers
  ros::Publisher activation_states_pub;

	//Subscribers
	ros::Subscriber obss_sub;

	//Service handler
	ros::ServiceClient update_occupancy_grid_srv;

	float time_before_initialisation;

	//Handle the objects
	std::vector<geometry_msgs::Vector3> recieved_obss;
	bool obss_recieved;
	float obss_size;


	Initialisation()
	{
		//Parameters input
		n.param<bool>("/initialisation/use_mapping_mode", mapping_mode, false);
		n.param<float>("/initialisation/time_before_starting_everything", time_before_initialisation, 5.0);
		n.param<int>("/initialisation/which_map_mode", map_mode, 1);

		//Publishers
		activation_states_pub = n.advertise<robo7_msgs::activation_states>("/robot_state/activation_states", 1);

		//Subscribers
		obss_sub = n.subscribe("/localization/mapping/the_obstacles", 1, &Initialisation::ObssCallback, this);

		//Service handler
		update_occupancy_grid_srv = n.serviceClient<robo7_srvs::UpdateOccupancyGridFiltered>("/occupancy_grid/update_occupancy_grid");

		bool obss_recieved = false;

	}

	void first_publish()
	{
		//Robot actions state
		state_activated.header.seq = 0;
		state_activated.header.stamp = ros::Time::now();
		state_activated.follow_point = false;
		state_activated.localize_itself = false;
		state_activated.mapping = true;

		activation_states_pub.publish( state_activated );
	}


	void ObssCallback(const robo7_msgs::allObstacles::ConstPtr &allobss){
		int num_obss = allobss->the_obstacles.size();

		if (num_obss > 0){
			obss_recieved = true;
			recieved_obss.clear();

			for (int i = 0; i < num_obss; ++i){
				recieved_obss.push_back(allobss->the_obstacles[i]);
			}

			obss_size = allobss->obstacle_size;

		}
	}

	void initialisation_sequence()
	{
		header_initialisation();
		if(!mapping_mode)
		{
			//Here should stand the initialisation for the non mapping mode
			//Otherwise it is the picking up sequence mode
			initialize_pickingup_states();
			updateOccupancyGridCall();

		}
		else
		{
			//Here stands the initialisation for the mapping mode

			//Start the mapping mode
			if(map_mode == 0)
			{
				initialize_mapping_states1();
			}
			else
			{
				initialize_mapping_states2();
			}

		}

		//Publish the initial state of the robot
		activation_states_pub.publish( state_activated );
	}

private:
	//Initialisation published_msgs
	robo7_msgs::activation_states state_activated;

	//The mode we choose
	bool mapping_mode;
	int map_mode;

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

	void updateOccupancyGridCall()
	{
		robo7_msgs::wallPoint no_point;
		no_point.number = 0;
		robo7_srvs::UpdateOccupancyGridFiltered::Request srv_req;
		robo7_srvs::UpdateOccupancyGridFiltered::Response srv_resp;

		robo7_msgs::allObstacles the_obs;
		the_obs.number = recieved_obss.size();
		the_obs.obstacle_size = obss_size;
		the_obs.the_obstacles = recieved_obss;


		srv_req.new_points = no_point;
		srv_req.the_obstacles = the_obs;

		update_occupancy_grid_srv.call(srv_req, srv_resp);
	}

	void initialize_mapping_states1()
	{
		//Robot state

		//Robot actions state
		state_activated.follow_point = false;
		state_activated.localize_itself = false;
		state_activated.mapping = true;
	}

	void initialize_mapping_states2()
	{
		//Robot state

		//Robot actions state
		state_activated.follow_point = false;
		state_activated.localize_itself = true;
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
