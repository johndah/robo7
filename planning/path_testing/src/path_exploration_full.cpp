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
#include <robo7_srvs/exploreTheMap.h>
#include <robo7_srvs/replaceExplorationPoints.h>
#include <robo7_srvs/distanceTo.h>
#include <robo7_srvs/moveToNextPoint.h>
#include <robo7_srvs/GoTo.h>

float control_frequency = 30.0;

class Exploration
{
public:
	ros::NodeHandle n;
	ros::Subscriber the_robot_pose_sub;
	ros::ServiceServer exploration_server;
	ros::ServiceClient replace_exploration_srv, move_to_next_point_srv, go_to_srv;

	Exploration()
	{
		//Service server
		exploration_server = n.advertiseService("/path_planning/explore", &Exploration::exploreSequence, this);

		//Subscribe the services
		replace_exploration_srv = n.serviceClient<robo7_srvs::replaceExplorationPoints>("/path_planning/replace_exploration_points");
		move_to_next_point_srv = n.serviceClient<robo7_srvs::moveToNextPoint>("/path_planning/move_to_next_point");
		go_to_srv = n.serviceClient<robo7_srvs::GoTo>("/kinematics/go_to");

	}

	bool exploreSequence(robo7_srvs::exploreTheMap::Request &req,
         robo7_srvs::exploreTheMap::Response &res)
	{
		robo7_srvs::replaceExplorationPoints::Request req1;
		robo7_srvs::replaceExplorationPoints::Response res1;
		replace_exploration_srv.call(req1, res1);

		robo7_srvs::moveToNextPoint::Request req2;
		robo7_srvs::moveToNextPoint::Response res2;
		move_to_next_point_srv.call(req2, res2);

		while(!res2.mapping_over)
		{
			replace_exploration_srv.call(req1, res1);
			move_to_next_point_srv.call(req2, res2);
		}

		robo7_srvs::GoTo::Request req3;
		robo7_srvs::GoTo::Response res3;
		req3.destination_pose = req.initial_pose;
		go_to_srv.call(req3, res3);

		res.success = true;
	}

private:

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ExplorationNode");

	Exploration Exploration_;

	ros::Rate loop_rate(control_frequency);

	ROS_INFO("Exploration Nodes is turning");

	ros::spin();

	return 0;
}
