#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>

// std includes
#include <limits>
#include <iostream>
#include <fstream>

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
		n.param<std::string>("/initialisation/obss_file", obss_file, "obss.txt");
		n.param<std::string>("/initialisation/walls_file", walls_file, "walls.txt");

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
			robo7_msgs::allObstacles the_obss = plot_batteries();
			robo7_msgs::wallPoint the_points = plot_points();
			updateOccupancyGridCall( the_obss , the_points );

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
	std::string obss_file, walls_file;

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

	void updateOccupancyGridCall(  robo7_msgs::allObstacles the_obstacles_msg , robo7_msgs::wallPoint the_points )
	{
		robo7_msgs::wallPoint no_point;
		no_point.number = 0;
		robo7_srvs::UpdateOccupancyGridFiltered::Request srv_req;
		robo7_srvs::UpdateOccupancyGridFiltered::Response srv_resp;

		robo7_msgs::allObstacles the_obs;
		the_obs.number = recieved_obss.size();
		the_obs.obstacle_size = obss_size;
		the_obs.the_obstacles = recieved_obss;


		srv_req.new_points = the_points;
		srv_req.the_obstacles = the_obstacles_msg;

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

	robo7_msgs::allObstacles plot_batteries()
	{
		robo7_msgs::allObstacles obss_msg;
		std::vector<geometry_msgs::Vector3> obstacles;

		// obstacle code starts here
		std::string line;
    std::ifstream obstacle_fs;
    obstacle_fs.open(obss_file.c_str());

    if (!obstacle_fs.is_open()){
        ROS_WARN("Could not read obstacles file, exploration mode?");
				obss_msg.number = 0;
				obss_msg.obstacle_size = 0;
				obstacles.clear();
				obss_msg.the_obstacles = obstacles;

				return obss_msg;
    } else{
      ROS_INFO("Reading obstacles from file");
      int num_obss = 0;
      float obss_size = 0;

      while (getline(obstacle_fs, line)){
          if (line[0] == '#') {
            // comment -> skip
            continue;
          }
          num_obss++;


          float max_num = std::numeric_limits<float>::max();

          float obs_size = max_num,
                 obs_x = max_num,
                 obs_y = max_num;

          std::istringstream line_stream(line);

          line_stream >> obs_size >> obs_x >> obs_y;

          if ((obs_size == max_num) || ( obs_x == max_num) || (obs_y == max_num)){
              ROS_WARN("Segment error in obss file. Skipping line: %s",line.c_str());
              continue;
          }

          obss_size = obs_size;
          geometry_msgs::Vector3 a_obstacle;
          a_obstacle.x = obs_x;
          a_obstacle.y = obs_y;

          obstacles.push_back(a_obstacle);
      }

      obss_msg.number = num_obss;
      obss_msg.obstacle_size = obss_size;
      obss_msg.the_obstacles = obstacles;

    }
		// obstacle code ends here
		return obss_msg;
	}

	robo7_msgs::wallPoint plot_points()
	{
		robo7_msgs::wallPoint point_msg;

		point_msg.number = 0;
		point_msg.the_points.clear();

		// obstacle code starts here
		std::string line;
    std::ifstream point_fs;
    point_fs.open(walls_file.c_str());

    if (!point_fs.is_open()){
        ROS_WARN("Could not read walls file, exploration mode?");
				return point_msg;
    } else{
      ROS_INFO("Reading obstacles from file");

      while (getline(point_fs, line)){
          if (line[0] == '#') {
            // comment -> skip
            continue;
          }

          float max_num = std::numeric_limits<float>::max();

          float  point_x = max_num,
                 point_y = max_num;

          std::istringstream line_stream(line);

          line_stream >> point_x >> point_y;

          if (( point_x == max_num) || (point_y == max_num)){
              ROS_WARN("Segment error in walls file. Skipping line: %s",line.c_str());
              continue;
          }

          geometry_msgs::Vector3 aPoint;
          aPoint.x = point_x;
          aPoint.y = point_y;

					point_msg.number++;
					point_msg.the_points.push_back( aPoint );
      }
    }

		return point_msg;
	}

	void call_update_grid( robo7_msgs::allObstacles the_obstacles_msg )
	{
		robo7_srvs::UpdateOccupancyGridFiltered::Request req1;
		robo7_srvs::UpdateOccupancyGridFiltered::Response res1;
		req1.the_obstacles = the_obstacles_msg;


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
