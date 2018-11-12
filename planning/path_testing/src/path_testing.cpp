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
												// 2.20, 2.20, 2.20, 2.20, 2.20, 2.20, 2.20, 2.20, 2.20,
												// 2.20, 2.20, 2.20, 2.20, 2.20, 2.20,
												// 2.19, 2.18, 2.10, 2.00, 1.90, 1.80, 1.70,
												// 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5,
												// 0.40, 0.32, 0.25, 0.22, 0.21, 0.20,
												// 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
												// 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
												// 0.22, 0.25, 0.30, 0.38, 0.45, 0.52, 0.6, 0.68, 0.75,
												// 0.80, 0.85,
												// 0.90, 0.95, 1.02, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60, 1.70, 1.73, 1.75, 1.76, 1.76, 1.75, 1.70,
												1.65, 1.60, 1.55, 1.50, 1.45, 1.40, 1.35, 1.30, 1.25,
												1.20,	1.15, 1.10, 1.05, 1.02, 1.01, 1.03, 1.08, 1.15, 1.20,
												1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.55, 1.60, 1.65, 1.70, 1.75,
												1.73, 1.71, 1.69, 1.67, 1.65, 1.63, 1.61, 1.59, 1.58, 1.58, 1.59, 1.60, 1.61, 1.62, 1.63, 1.64,
												1.50, 1.35, 1.20, 1.05, 0.90, 0.80,
												0.70, 0.64, 0.62, 0.60, 0.60, 0.60, 0.60, 0.61, 0.63, 0.67, 0.73, 0.80,
												0.90, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60, 1.70, 1.80, 1.90, 2.00,
												};

		float Y_coord[] = {robot_position.linear.y,
												// 0.50, 0.60, 0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30,
												// 1.40, 1.50, 1.60, 1.70, 1.80, 1.90,
												// 2.00, 2.10, 2.15, 2.18, 2.19, 2.20, 2.20,
												// 2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2, 2.2,
												// 2.18, 2.15, 2.10, 2.00, 1.90, 1.80, 1.70,
												// 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2,
												// 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2,
												// 1.30, 1.40, 1.45, 1.49, 1.50, 1.49, 1.44, 1.36, 1.25,
												// 1.15, 1.05,
												// 0.98, 0.93, 0.88, 0.84, 0.82, 0.82, 0.82, 0.82, 0.82, 0.82, 0.83, 0.84, 0.87, 0.90, 0.93, 1.00,
												1.07, 1.14, 1.21, 1.28, 1.35, 1.42, 1.49, 1.56, 1.63,
												1.68,	1.71, 1.72, 1.71, 1.68, 1.63, 1.58, 1.55, 1.56, 1.59,
												1.62, 1.65, 1.68, 1.71, 1.74, 1.77, 1.80, 1.83, 1.86, 1.89, 1.92,
												1.85, 1.78, 1.71, 1.64, 1.57, 1.50, 1.43, 1.36, 1.29, 1.22, 1.15, 1.08, 1.01, 0.94, 0.87, 0.80,
												0.82, 0.84, 0.86, 0.88, 0.91, 0.92,
												0.90, 0.86, 0.81, 0.75, 0.70, 0.65, 0.60, 0.50, 0.42, 0.36, 0.32, 0.30,
												0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28,

												};

		std::vector<float> X_coordinates (X_coord, X_coord + sizeof(X_coord) / sizeof(float) );
		std::vector<float> Y_coordinates (Y_coord, Y_coord + sizeof(Y_coord) / sizeof(float) );



		for(unsigned i=0; i<X_coordinates.size(); i++)
		{
			aPoint.id_number = i;
			// ROS_INFO("(x,y) : %lf, %lf", X_coordinates.at(i), Y_coordinates.at(i));
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
