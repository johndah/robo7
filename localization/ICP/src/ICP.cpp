#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Geometry>

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
		pi = 3.14159265358979323846;

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
		former_point = req.current_position;
		map_corner_list = req.the_wall_corners;
		lidar_corner_list = req.the_lidar_corners;

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

		//icp.setRANSACOutlierRejectionThreshold(5);
		icp.setRANSACOutlierRejectionThreshold(0.5);
    //icp.setRANSACIterations(100);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (1);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (1000);

    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (0.001);
		// icp.setTransformationRotationEpsilon(0.05);
    //std::cout << " getTransformationEpsilon epsilon: "<<icp.getTransformationEpsilon() << std::endl;
    //std::cout << " getEuclideanFitnessEpsilon epsilon: "<<icp.getEuclideanFitnessEpsilon() << std::endl;
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.001);

	  icp.setInputSource(cloud_lidar);
	  icp.setInputTarget(cloud_map);
	  icp.align(Final);
		converged = icp.hasConverged();
		error = icp.getFitnessScore();
		transformation_ = icp.getFinalTransformation();

		//Extract the robot position in the Vector4
		forwardTransform();

		//Transform the points with the icp transformation found
		transform_points();

		//Update the pose of the robot
		inverseTransform();



		res.success = converged;
		res.error = error;
		res.new_position = new_point;
		for(int i=0; i<transformation_.cols(); i++)
		{
			res.transformation.line0.push_back(transformation_(0,i));
			res.transformation.line1.push_back(transformation_(1,i));
			res.transformation.line2.push_back(transformation_(2,i));
			res.transformation.line3.push_back(transformation_(3,i));
		}

		corrected_pos_pub.publish( new_point );

		return true;
	}

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map;
	pcl::PointCloud<pcl::PointXYZ> Final;

	robo7_msgs::cornerList lidar_corner_list;
	robo7_msgs::cornerList map_corner_list;

	Eigen::Matrix4f transformation_;
	float error;
	bool converged;

	geometry_msgs::Twist former_point;
	geometry_msgs::Twist new_point;

	Eigen::Vector4f the_point_;
	Eigen::Vector4f other_point_;

	float robot_angle_;
	float pi;

	float findangle(float x, float y)
	{
	  if(x==0)
	  {
	    return pi*sgn(y);
	  }
	  else if((x<0)&&(y>0))
	  {
	    return atan(y/x) + pi;
	  }
	  else if ((x<0)&&(y<0))
	  {
	    return atan(y/x) - pi;
	  }
	  else
	  {
	    return atan(y/x);
	  }
	  // return atan(x/y);
	}

	void forwardTransform()
	{
		the_point_(0) = former_point.linear.x;
		the_point_(1) = former_point.linear.y;
		the_point_(2) = former_point.linear.z;
		the_point_(3) = 1;
		robot_angle_ = former_point.angular.z;

		other_point_(0) = the_point_(0) + cos(robot_angle_);
		other_point_(1) = the_point_(1) + sin(robot_angle_);
		other_point_(2) = the_point_(2);
		other_point_(3) = 1;
	}

	void transform_points()
	{
		the_point_ = transformation_ * the_point_;
		other_point_ = transformation_ * other_point_;
	}

	void inverseTransform()
	{
		ROS_INFO("%lf", wrapAngle( robot_angle_ ));
		robot_angle_ = findangle(other_point_(0) - the_point_(0), other_point_(1) - the_point_(1));

		new_point.linear.x = the_point_(0);
		new_point.linear.y = the_point_(1);
		new_point.linear.z = the_point_(2);
		new_point.angular.x = 0;
		new_point.angular.y = 0;
		new_point.angular.z = wrapAngle( robot_angle_ );

		ROS_INFO("%lf, %lf, %lf, %lf", the_point_(0), the_point_(1), the_point_(2), wrapAngle( robot_angle_ ));
	}

	int sgn(float v)
	{
		if (v < 0) return -1;
		else if (v > 0) return 1;
		else return 0;
	}

	float wrapAngle( double angle )
	{
		float twoPi = 2.0 * pi;
		return angle - twoPi * floor( angle / twoPi );
	}
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
