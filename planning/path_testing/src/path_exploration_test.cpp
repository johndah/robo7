#include <unistd.h>

#include <ros/ros.h>

//Messages
#include <robo7_msgs/the_robot_position.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallPoint.h>
#include <robo7_msgs/cornerList.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

//Service
#include <robo7_srvs/IsGridOccupied.h>
#include <robo7_srvs/replaceExplorationPoints.h>
#include <robo7_srvs/distanceTo.h>
#include <robo7_srvs/moveToNextPoint.h>
#include <robo7_srvs/GoTo.h>

float control_frequency = 30.0;

class PathExplorer
{
public:
	ros::NodeHandle n;
	ros::Subscriber the_robot_pose_sub;
	ros::ServiceServer path_exploration_service, move_to_next_point_service;
  ros::Publisher the_unexplored_points_pub, the_explored_points_pub, the_aimed_point_pub;
	ros::ServiceClient is_cell_occupied_srv, distance_to_srv, go_to_srv;

	PathExplorer()
	{
		//The initial parameters
		n.param<float>("/path_exploration_test/dist_between_points", dist_between_points, 0.50);
		n.param<float>("/path_exploration_test/close_enough_to_goal", close_enough_threshold, 0.25);

		//Define the subscribers
		the_robot_pose_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &PathExplorer::robot_pose_callBack, this);

		//Define the services
		path_exploration_service = n.advertiseService("/path_planning/replace_exploration_points", &PathExplorer::replaceAllPoints, this);
		move_to_next_point_service = n.advertiseService("/path_planning/move_to_next_point", &PathExplorer::moveToNextPoint, this);

		//Subscribe the services
		is_cell_occupied_srv = n.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
		distance_to_srv = n.serviceClient<robo7_srvs::distanceTo>("/distance_grid/distance");
		go_to_srv = n.serviceClient<robo7_srvs::GoTo>("/kinematics/go_to");

		//Define the publishers
		the_unexplored_points_pub = n.advertise<robo7_msgs::wallPoint>("/path_planning/exploration/the_unexplored_points", 1);
		the_explored_points_pub = n.advertise<robo7_msgs::wallPoint>("/path_planning/exploration/the_explored_points", 1);
		the_aimed_point_pub = n.advertise<geometry_msgs::Vector3>("/path_planning/exploration/the_goal_point", 1);

		//Initialisation
		x_max = 2.40;
		y_max = 2.40;
		create_looking_around();
		create_the_map_of_points();
	}

	void robot_pose_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;
  }

	void publishPoints()
	{
		the_unexplored_points_pub.publish( allUnexploredPoints );
		the_explored_points_pub.publish( allExploredPoints );
	}

	void create_the_map_of_points()
	{
		//Define the number of points to follow
		int nb_x = (int)(x_max/dist_between_points);
		int nb_y = (int)(y_max/dist_between_points);

		//Define the x and y of each points
		geometry_msgs::Vector3 onePoint;
		float x_step = x_max/nb_x;
		float y_step = y_max/nb_y;
		for(int i=0; i < nb_x+1; i++)
		{
			for(int j=0; j < nb_y+1; j++)
			{
				if((i>0)||(j>0))
				{
					onePoint.x = i*x_step;
					onePoint.y = j*y_step;
					onePoint.z = 0;

					allUnexploredPoints.number++;
					allUnexploredPoints.the_points.push_back( onePoint );
				}
			}
		}

		for(int i=0; i<10; i++)
		{
			publishPoints();
		}
	}

	void create_looking_around()
	{
		i_index.clear();
		j_index.clear();
		int cell_around = 20;

		for(int k=1; k<cell_around; k++)
		{
			i_index.push_back(k); i_index.push_back(0); i_index.push_back(-k); i_index.push_back(0);
			j_index.push_back(0); j_index.push_back(k); j_index.push_back(0); j_index.push_back(-k);
		}
		for(int k=1; k<cell_around; k++)
		{
			i_index.push_back(k); i_index.push_back(k); i_index.push_back(-k); i_index.push_back(-k);
			j_index.push_back(-k); j_index.push_back(k); j_index.push_back(k); j_index.push_back(-k);
		}
	}

	void replace_the_points(robo7_msgs::wallPoint theRestPoint)
	{
		//Initialisation
		allUnexploredPoints.number = 0;
		allUnexploredPoints.the_points.clear();
		robo7_srvs::IsGridOccupied::Request req1;
		robo7_srvs::IsGridOccupied::Response res1;
		float cell_size = 0.02;

		//Recreating the unexplored cells
		for(int i=0; i < theRestPoint.number; i++)
		{
			float local_x = theRestPoint.the_points[i].x;
			float local_y = theRestPoint.the_points[i].y;
			req1.x = local_x;
			req1.y = local_y;
			if((local_x > 0)&&(local_x < x_max)&&(local_y > 0)&&(local_y < y_max))
			{
				is_cell_occupied_srv.call(req1, res1);
			}
			else
			{
				res1.occupancy = 1;
			}
			// ROS_INFO("Is occupied : %lf", res1.occupancy);
			if(res1.occupancy == 1)
			{
				// ROS_INFO("Init (x,y) = (%lf, %lf)", local_x, local_y);
				//Replace all the points
				for(int k=0; k < static_cast<int>(i_index.size()); k++)
				{
				  local_x = theRestPoint.the_points[i].x + i_index[k]*cell_size;
					local_y = theRestPoint.the_points[i].y + j_index[k]*cell_size;
					if((local_x > 0)&&(local_x < x_max)&&(local_y > 0)&&(local_y < y_max))
					{
						// ROS_INFO("(x,y) = (%lf, %lf)", local_x, local_y);
						//Call the service to check if this place is occupied
						req1.x = local_x;
						req1.y = local_y;
						is_cell_occupied_srv.call(req1, res1);
						// ROS_INFO("Is occupied : %lf", res1.occupancy);
						if(res1.occupancy != 1)
						{
							theRestPoint.the_points[i].x = local_x;
							theRestPoint.the_points[i].y = local_y;
							allUnexploredPoints.number++;
							allUnexploredPoints.the_points.push_back( theRestPoint.the_points[i] );
							break;
						}
					}
				}
			}
			else
			{
				allUnexploredPoints.number++;
				allUnexploredPoints.the_points.push_back( theRestPoint.the_points[i] );
			}
		}
	}

	bool replaceAllPoints(robo7_srvs::replaceExplorationPoints::Request &req,
         robo7_srvs::replaceExplorationPoints::Response &res)
	{

		replace_the_points( allUnexploredPoints );

		publishPoints();

		res.success = true;
	}

	bool moveToNextPoint(robo7_srvs::moveToNextPoint::Request &req,
         robo7_srvs::moveToNextPoint::Response &res)
	{
		//First we need to choose which one is the next point
		robo7_srvs::distanceTo::Request req1;
		robo7_srvs::distanceTo::Response res1;

		//Distance from the initial point
		ros::spinOnce();
		req1.x_to = the_robot_pose.position.linear.x;
		req1.y_to = the_robot_pose.position.linear.y;

		//Looking for the closest destination point
		int point_index = 0;
		int distance_to_start = 0;

		if(allUnexploredPoints.number > 0)
		{
			req1.x_from = allUnexploredPoints.the_points[0].x;
			req1.y_from = allUnexploredPoints.the_points[0].y;
			distance_to_srv.call(req1, res1);
			distance_to_start = res1.distance;
		}
		else
		{
			res.mapping_over = true;
			return true;
		}

		for(int i=1; i < allUnexploredPoints.number; i++)
		{
			// ROS_INFO("The closest point is %d, about %d cm", point_index, distance_to_start);
			// ROS_INFO("robot (%lf, %lf), point (%lf, %lf)", req1.x_to, req1.y_to, req1.x_from, req1.y_from);
			req1.x_from = allUnexploredPoints.the_points[i].x;
			req1.y_from = allUnexploredPoints.the_points[i].y;
			distance_to_srv.call(req1, res1);
			if(res1.distance < distance_to_start)
			{
				distance_to_start = res1.distance;
				point_index = i;
			}
		}

		geometry_msgs::Vector3 point_to_follow = allUnexploredPoints.the_points[point_index];

		the_aimed_point_pub.publish( point_to_follow );

		geometry_msgs::Twist destination;
		destination.linear.x = point_to_follow.x;
		destination.linear.y = point_to_follow.y;
		destination.linear.z = point_to_follow.z;
		destination.angular.z = -1;

		//Do go_To service and see what it does return
		robo7_srvs::GoTo::Request req2;
		robo7_srvs::GoTo::Response res2;
		req2.destination_pose = destination;
		go_to_srv.call(req2, res2);

		if(res2.success||distanceToGoal(destination))
		{
			move_unexplored_point_to_explored_point( point_index );
		}

		publishPoints();

		res.mapping_over = false;
		return true;
	}

	bool distanceToGoal(geometry_msgs::Twist destination)
	{
		ros::spinOnce();
		float x1 = the_robot_pose.position.linear.x;
		float y1 = the_robot_pose.position.linear.y;
		float x2 = destination.linear.x;
		float y2 = destination.linear.y;

		float dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

		return (dist < close_enough_threshold);
	}

	void move_unexplored_point_to_explored_point(int point_index)
	{
		robo7_msgs::wallPoint copyUnexplored = allUnexploredPoints;
		allUnexploredPoints.number = 0;
		allUnexploredPoints.the_points.clear();

		for(int i=0; i < copyUnexplored.number; i++)
		{
			if( i == point_index )
			{
				allExploredPoints.number++;
				allExploredPoints.the_points.push_back( copyUnexplored.the_points[i] );
			}
			else
			{
				allUnexploredPoints.number++;
				allUnexploredPoints.the_points.push_back( copyUnexplored.the_points[i] );
			}
		}
	}

private:
	//Subscribers variables
	robo7_msgs::the_robot_position the_robot_pose;

	//Inner variables
	robo7_msgs::wallPoint allUnexploredPoints;
	robo7_msgs::wallPoint allExploredPoints;

	//Initial variables
	float dist_between_points;
	std::vector<int> i_index;
	std::vector<int> j_index;

  //Variables definition
	float x_max, y_max;
	float close_enough_threshold;

	//

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ExplorationNode");

	PathExplorer PathExplorer_;

	ros::Rate loop_rate(control_frequency);

	ROS_INFO("Exploration Nodes is turning");

	PathExplorer_.publishPoints();

	ros::spin();

	return 0;
}
