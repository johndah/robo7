#include <ros/ros.h>

//The messages
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/cornerList.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

//The services
#include <robo7_srvs/RansacWall.h>
#include <robo7_srvs/update_map.h>
#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/discretize_map.h>

#include <stdlib.h>

#include <string>
#include <vector>


class UpdateMap
{
public:
	ros::NodeHandle n;
	ros::ServiceServer update_map_server;
	ros::ServiceClient scan_to_coord_srv;
	ros::ServiceClient discretized_map_srv;
  ros::ServiceClient ransac_srv;
  ros::Publisher former_map_pub;
	ros::Publisher lidar_map_pub;
	ros::Publisher new_map_pub;

	UpdateMap()
	{
		n.param<int>("/update_map/choose_method", method, 0);
		// pi = 3.14159265358979323846;

		//ServiceServer
		update_map_server = n.advertiseService("/localization/mapping/update_map", &UpdateMap::update_Sequence, this);

		//Service Clients
		ransac_srv = n.serviceClient<robo7_srvs::RansacWall>("/localization/ransac");
		scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
		discretized_map_srv = n.serviceClient<robo7_srvs::discretize_map>("/maze_map/map_discretization");

		//Visualization of the updated versions of the walls
		former_map_pub = n.advertise<robo7_msgs::wallList>("/localization/mapping/former_map", 1);
		lidar_map_pub = n.advertise<robo7_msgs::wallList>("/localization/mapping/lidar_map", 1);
		new_map_pub = n.advertise<robo7_msgs::wallList>("/localization/mapping/new_map", 1);
	}

	bool update_Sequence(robo7_srvs::update_map::Request &req,
         robo7_srvs::update_map::Response &res)
	{
		// former_point = req.current_position;
		the_lidar_scan = req.the_lidar_scan;
		the_map_walls = req.map_walls;
		robot_position = req.robot_position;

		//Discretize the map walls message
		discretize_world_map();
		ROS_INFO("World map discretized");

		//Extract the lidar point cloud
		extract_lidar_cloud();
		ROS_INFO("Lidar cloud extracted");

		//Two methods
		if(method == 0)
		{
			//Method consisting in extracting the walls from the lidar cloud
			//and then discretize the lidar walls (weight of nb inliers)
			ransac_for_lidar();
			lidar_map_pub.publish( lidar_walls );
			ROS_INFO("blu");
		}
		else
		{
			//Method consisting in directly use the lidar scan and merge it with the
			//discretize map walls (weight given by one point -> weight one)
			discretized_lidar_map = lidar_cloud;
		}

		//Once the different set of points defined, it is needed to merge the two point
		//clouds together -> carefull because the inliers won't be sorted so it is
		//needed to do it when it comes up with the creation of the walls
		merge_the_point_clouds();
		ROS_INFO("Clouds merged");

		//Recreate the whole final map
		final_ransac_walls();
		ROS_INFO("New map updated");

		//Publish some messages for the visualization part
		former_map_pub.publish(the_map_walls);
		new_map_pub.publish(updated_map_walls);

		res.success = true;
		res.updated_map_walls = updated_map_walls;

		return true;
	}

private:
	//Input and output of the main service
	sensor_msgs::LaserScan the_lidar_scan;
	robo7_msgs::wallList the_map_walls;
	robo7_msgs::former_position robot_position;
	robo7_msgs::wallList updated_map_walls;

	//Discretized world map and lidar map
	robo7_msgs::wallPoint discretized_world_map;
	robo7_msgs::wallPoint discretized_lidar_map;

	//The lidar point cloud extracted
	robo7_msgs::wallPoint lidar_cloud;
	robo7_msgs::wallList lidar_walls;

	//Final ransac msgs
	robo7_msgs::wallPoint merged_clouds;

	//Choose method
	int method;



	void discretize_world_map()
	{
		robo7_srvs::discretize_map::Request req1;
		robo7_srvs::discretize_map::Response res1;
		req1.walls = the_map_walls;
		discretized_map_srv.call(req1, res1);
		discretized_world_map = res1.discretized_walls;
	}

	void extract_lidar_cloud()
	{
		robo7_srvs::scanCoord::Request req1;
		robo7_srvs::scanCoord::Response res1;
		req1.robot_position = robot_position.position;
		req1.lidar_scan = the_lidar_scan;
		scan_to_coord_srv.call(req1, res1);
		lidar_cloud = res1.the_lidar_points;
	}

	void ransac_for_lidar()
	{
		//Find the walls
		robo7_srvs::RansacWall::Request req1;
		robo7_srvs::RansacWall::Response res1;
		req1.the_cloud = lidar_cloud;
		ransac_srv.call(req1, res1);
		lidar_walls = res1.ransac_walls;

		//Discretize those walls
		robo7_srvs::discretize_map::Request req2;
		robo7_srvs::discretize_map::Response res2;
		req2.walls = lidar_walls;
		discretized_map_srv.call(req2, res2);
		discretized_lidar_map = res2.discretized_walls;
	}

	void merge_the_point_clouds()
	{
		merged_clouds.number = 0;
		merged_clouds.the_points.clear();

		//Add the points of the map cloud
		for(int i = 0; i < discretized_world_map.number; i++)
		{
			merged_clouds.the_points.push_back(discretized_world_map.the_points[i]);
			merged_clouds.number++;
		}
		//Add the points of the lidar cloud
		for(int i = 0; i < discretized_lidar_map.number; i++)
		{
			merged_clouds.the_points.push_back(discretized_lidar_map.the_points[i]);
			merged_clouds.number++;
		}

		ROS_INFO("Number of points (prev_map, lidar, merged) : (%d, %d, %d)", discretized_world_map.number, discretized_lidar_map.number, merged_clouds.number);
	}

	void final_ransac_walls()
	{
		robo7_srvs::RansacWall::Request req1;
		robo7_srvs::RansacWall::Response res1;
		req1.the_cloud = merged_clouds;
		ransac_srv.call(req1, res1);
		updated_map_walls = res1.ransac_walls;
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mapping service");

	UpdateMap UpdateMap_;

	ros::Rate loop_rate(100);

	ROS_INFO("Mapping Service is running");

	ros::spin();

	return 0;
}