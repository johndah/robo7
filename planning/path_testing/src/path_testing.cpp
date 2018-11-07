#include <unistd.h>

#include <ros/ros.h>
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_srvs/path_planning.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


class PathTester
{
public:
	ros::NodeHandle n;
	ros::ServiceServer path_service;
  ros::Publisher path_pub;

	PathTester()
	{
		path_service = n.advertiseService("/path_planning/path_testing", &PathTester::path_Sequence, this);
    path_pub = n.advertise<robo7_msgs::trajectory>("/path_planning/path_testing/trajectory", 1);
	}

	bool path_Sequence(robo7_srvs::path_planning::Request &req,
         robo7_srvs::path_planning::Response &res)
	{
	  // ROS_INFO("Starting Ransac");

		geometry_msgs::Twist robot_position = req.robot_position;
		robo7_msgs::trajectory_point aPoint;
		robo7_msgs::trajectory all_points;

		//The coordinates of the path to test
		float X_coord[] = {robot_position.linear.x,
												2, 1, 1.05};
		float Y_coord[] = {robot_position.linear.y,
												2, 0, 1.7};

		std::vector<int> X_coordinates (X_coord, X_coord + sizeof(X_coord) / sizeof(float) );
		std::vector<int> Y_coordinates (Y_coord, Y_coord + sizeof(Y_coord) / sizeof(float) );



		for(unsigned i=0; i<X_coordinates.size(); i++)
		{
			aPoint.id_number = i;
			aPoint.point_coord.x = X_coordinates.at(i);
			aPoint.point_coord.y = Y_coordinates.at(i);
			aPoint.point_coord.z = 0;
			all_points.trajectory_points.push_back(aPoint);
		}

		path_pub.publish( all_points );

		res.path_planned = all_points;
		res.success = true;

	}


private:
  robo7_msgs::XY_coordinates point_cloud_XY_2;

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Path_testing");

	PathTester PathTester_;

	ros::Rate loop_rate(100);

	ROS_INFO("Path_testing to find walls running");

	ros::spin();

	return 0;
}
