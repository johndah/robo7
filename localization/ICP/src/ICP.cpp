#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <unistd.h>

#include <ros/ros.h>
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_srvs/ICPAlgorithm.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <stdlib.h>

#include <string>
#include <vector>

// boost::shared_ptr<pcl::visualization::PCLVisualizer>
//
// simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   //viewer->addCoordinateSystem (1.0, "global");
//   viewer->initCameraParameters ();
//   return (viewer);
// }

class ICPServer
{
public:
	ros::NodeHandle n;
	ros::Subscriber corner_map_sub;
	ros::Subscriber corner_lidar_sub;
	ros::Subscriber current_pos_sub;
	ros::ServiceServer ICP_service;
  ros::Publisher corrected_pos_pub;

	ICPServer()
	{
		// n.param<float>("/ransac/threshold", ransac_threshold, 0.01);

		corner_map_sub = n.subscribe("/own_map/map_corners", 1, &ICPServer::map_corners_Callback, this);
		corner_lidar_sub = n.subscribe("/localization/ransac/corners", 1, &ICPServer::lidar_corners_Callback, this);
		ICP_service = n.advertiseService("/localization/icp", &ICPServer::ICPSequence, this);
    corrected_pos_pub = n.advertise<geometry_msgs::Twist>("/localization/icp/position", 1);
	}

	void map_corners_Callback(const robo7_msgs::cornerList::ConstPtr &msg)
	{
    map_corner_list.number = msg->number;
		map_corner_list.corners = msg->corners;
	}

	void lidar_corners_Callback(const robo7_msgs::cornerList::ConstPtr &msg)
	{
		lidar_corner_list.number = msg->number;
		lidar_corner_list.corners = msg->corners;
	}

	bool ICPSequence(robo7_srvs::ICPAlgorithm::Request &req,
         robo7_srvs::ICPAlgorithm::Response &res)
	{

		cloud_lidar = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

		//Definition of the parameters of the cloud
		cloud_lidar->width    = lidar_corner_list.number;
		cloud_lidar->height   = 1;
		cloud_lidar->is_dense = false;
		cloud_lidar->points.resize (cloud_lidar->width * cloud_lidar->height);

		cloud_map->width    = map_corner_list.number;
		cloud_map->height   = 1;
		cloud_map->is_dense = false;
		cloud_map->points.resize (cloud_map->width * cloud_map->height);


		//Assesment of the corresponding variables
		for (size_t i = 0; i < cloud_lidar->points.size(); ++i)
	  {
	    cloud_lidar->points[i].x = lidar_corner_list.corners[i].x;
	    cloud_lidar->points[i].y = lidar_corner_list.corners[i].y;
	    cloud_lidar->points[i].z = lidar_corner_list.corners[i].z;
	  }

		for (size_t i = 0; i < cloud_map->points.size(); ++i)
	  {
	    cloud_map->points[i].x = map_corner_list.corners[i].x;
	    cloud_map->points[i].y = map_corner_list.corners[i].y;
	    cloud_map->points[i].z = map_corner_list.corners[i].z;
	  }

		//Solve ICP
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	  icp.setInputSource(cloud_lidar);
	  icp.setInputTarget(cloud_map);
	  icp.align(Final);
		// icp.hasConverged();
		// icp.getFitnessScore();
		// icp.getFinalTransformation();

		res.success = true;
		return true;
	}

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map;
	pcl::PointCloud<pcl::PointXYZ> Final;

	robo7_msgs::cornerList lidar_corner_list;
	robo7_msgs::cornerList map_corner_list;

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ICPServer");

	ICPServer icp_;

	ros::Rate loop_rate(100);

	ROS_INFO("ICP Algorithm to find walls running");

	ros::spin();

	return 0;
}
