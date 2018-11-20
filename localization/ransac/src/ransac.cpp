// #include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <boost/thread/thread.hpp>

#include <unistd.h>

#include <ros/ros.h>
#include <robo7_msgs/XY_coordinates.h>
#include <robo7_msgs/aWall.h>
#include <robo7_msgs/wallList.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_srvs/RansacWall.h>
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

class RansacServer
{
public:
	ros::NodeHandle n;
	ros::Subscriber point_cloud;
	ros::ServiceServer ransac_service;
  ros::Publisher model_pub;
	ros::Publisher wall_list;
  ros::Publisher corner_pub;

	RansacServer()
	{
		n.param<float>("/ransac/threshold", ransac_threshold, 0.01);
		n.param<int>("/ransac/minimum_point_for_a_wall", min_point, 5);
		n.param<float>("/ransac/maximum_distance_between_points", maximum_space_between_points, 1);
    n.param<int>("/ransac/minimum_point_for_a_gap", gap_threshold, 3);

    min_point = std::max(min_point, 5); //Security loop to avoid inifinity wall detection

		ransac_service = n.advertiseService("/localization/ransac", &RansacServer::ransacSequence, this);

		//Publishers
		model_pub = n.advertise<geometry_msgs::Twist>("/localization/ransac/modelLine", 1);
		wall_list = n.advertise<robo7_msgs::wallList>("/localization/ransac/walls", 1);
    corner_pub = n.advertise<robo7_msgs::cornerList>("/localization/ransac/corners", 1);
	}

	bool ransacSequence(robo7_srvs::RansacWall::Request &req,
         robo7_srvs::RansacWall::Response &res)
	{
		ROS_INFO("Start RANSAC");
		the_cloud = req.the_cloud;
		ROS_INFO("Number of points : %d, %lf", the_cloud.number, the_cloud.the_points[0].x);
		robo7_msgs::wallList all_the_walls;
    robo7_msgs::cornerList all_corners;

		wall_full_list.clear();
    the_corners_list.clear();


    // initialize PointClouds
		cloud_init = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    final_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

		//Initialize the wall extraction algorithm
		still_walls = true;

		bool copy = true;

    clean_the0points();
    wall_nb = 1;

		ROS_INFO("Extraction of the walls");
		//Extraction of all the walls
		while(still_walls)
		{
			ROS_INFO("Update cloud");
      updateCloud();

			if(copy)
			{
				pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *cloud_init);
				copy = false;
			}

			ROS_INFO("Solve RANSAC");
			ROS_INFO("Cloud size %d, %d", cloud->width, cloud->height);
			//Find the line model that fit this new point cloud
			if(cloud->width > 2) {
				solveRansac();
			} else {
				still_walls = false;
			}

			ROS_INFO("Add the walls");
			if( (static_cast<int>(inliers.size()) < min_point) || !still_walls )
			{
				still_walls = false;
			}
			else
			{
				//Extract the corresponding wall to the line model
				createWall();

				//Remove the inliers from the previously used point cloud in order
				//to look for another model (wall)
        updateInliers();
				updatePoints();
			}
		}

		all_the_walls.walls = wall_full_list;
		all_the_walls.number = wall_full_list.size();

    all_corners.number = the_corners_list.size();
    all_corners.corners = the_corners_list;

		wall_list.publish( all_the_walls );
    corner_pub.publish( all_corners );

		res.all_corners = all_corners;
		res.ransac_walls = all_the_walls;
		res.success = true;

		ROS_INFO("RANSAC done \n");
    return true;
	}

	void updateCloud()
	{
		// populate our PointCloud with points
    cloud->width    = the_cloud.the_points.size();
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

    for(int i=0; i<cloud->width; i++)
    {
      cloud->points[i].x = the_cloud.the_points[i].x;
      cloud->points[i].y = the_cloud.the_points[i].y;
      cloud->points[i].z = the_cloud.the_points[i].z;
    }

		// ROS_INFO("Number of points : %d, %lf", cloud->width, cloud->points[0].x);
	}

  void clean_the0points()
  {
    inliers.clear();
    for(int i=static_cast<int>(the_cloud.the_points.size())-1; i>-1; i--)
    {
      if((the_cloud.the_points[i].x == 0)&&(the_cloud.the_points[i].y == 0))
      {
        inliers.insert(inliers.begin(), i);
      }
      else
      {
        break;
      }
    }
    updatePoints();
  }

	void solveRansac()
	{
		pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
      model_p (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (ransac_threshold);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients( model_param_ );

		// ROS_INFO("inliers : %d", static_cast<int>(inliers.size()));
		// printInliers();

		x0 = model_param_(0); y0 = model_param_(1); z0 = model_param_(2);
		a = model_param_(3); b = model_param_(4); c = model_param_(5);
	}

  void printInliers()
  {
    for(int i=0; i<static_cast<int>(inliers.size()); i++)
    {
      ROS_INFO("%d", inliers[i]);
    }
  }

  void updateInliers()
  {
    //Leave for of the extremities point within the point cloud because it can belong to some other wall
    index1 = (index1)%static_cast<int>(inliers.size());
    if(index1==0)
    {
      inliers.erase(inliers.begin() + static_cast<int>(inliers.size()) - 1);
      inliers.erase(inliers.begin() + static_cast<int>(inliers.size()) - 1);
      inliers.erase(inliers.begin());
      inliers.erase(inliers.begin());
    }
    else
    {
      inliers.erase(inliers.begin() + (index1-2)%static_cast<int>(inliers.size()));
      inliers.erase(inliers.begin() + (index1-2)%static_cast<int>(inliers.size()));
      inliers.erase(inliers.begin() + (index1-2)%static_cast<int>(inliers.size()));
      inliers.erase(inliers.begin() + (index1-2)%static_cast<int>(inliers.size()));
    }
  }

	void updatePoints() //Looking for another unknown wall in the left data points (aka not inliers)
	{
		// ROS_INFO("The sizes : %d, %d", static_cast<int>(the_cloud.the_points.size()), static_cast<int>(inliers.size()));
    //Remove all the rest from the cloud
		for(int i=inliers.size()-1; i>-1; i--)
		{
			the_cloud.the_points.erase(the_cloud.the_points.begin() + inliers[i]);
		}
		// ROS_INFO("The sizes : %d", static_cast<int>(the_cloud.the_points.size()));
	}

  void extractExtremities()
  {
    x_min_ind = 0; x_max_ind = 0; y_min_ind = 0; y_max_ind = 0;
    for(int i=1; i<static_cast<int>(inliers.size()); i++)
    {
      if(the_cloud.the_points[inliers[x_min_ind]].x > the_cloud.the_points[inliers[i]].x)
      {
        x_min_ind = i;
      }
      if(the_cloud.the_points[inliers[x_max_ind]].x < the_cloud.the_points[inliers[i]].x)
      {
        x_max_ind = i;
      }
      if(the_cloud.the_points[inliers[y_min_ind]].y > the_cloud.the_points[inliers[i]].y)
      {
        y_min_ind = i;
      }
      if(the_cloud.the_points[inliers[y_max_ind]].y < the_cloud.the_points[inliers[i]].y)
      {
        y_max_ind = i;
      }
    }
  }

	void createWall()
	{
    extractExtremities();
    x1 = the_cloud.the_points[inliers[x_min_ind]].x;
    y1 = the_cloud.the_points[inliers[y_min_ind]].y;
    x2 = the_cloud.the_points[inliers[x_max_ind]].x;
    y2 = the_cloud.the_points[inliers[y_max_ind]].y;

		x_diff = std::abs(x1 - x2);
		y_diff = std::abs(y1 - y2);

    tot_inliers = static_cast<int>(inliers.size());

		//Extract the index we should start with
		extract_start_index();

    index1 = index;

    wall_done = true;
    wall_pts_nb = 0;
    for(int i=0; i<tot_inliers; i++)
    {
      if(x_diff > y_diff)
      {
        x_ext = cloud->points[inliers[index%tot_inliers]].x;
        y_ext = compute_y(x_ext);
        x_ext1 = cloud->points[inliers[(index+1)%tot_inliers]].x;
        y_ext1 = compute_y(x_ext1);
      }
      else
      {
        y_ext = cloud->points[inliers[index%tot_inliers]].y;
        x_ext = compute_x(y_ext);
        y_ext1 = cloud->points[inliers[(index+1)%tot_inliers]].y;
        x_ext1 = compute_x(y_ext1);
      }

      dist = sqrt(pow(x_ext - x_ext1, 2) + pow(y_ext - y_ext1, 2));
      if(wall_done)
      {
        single_wall.init_point.x = x_ext;
        single_wall.init_point.y = y_ext;
        single_wall.init_point.z = 0;
        wall_done = false;
        wall_pts_nb = 0;
      }
      if(i == tot_inliers - 1)
      {
        single_wall.end_point.x = x_ext;
        single_wall.end_point.y = y_ext;
        single_wall.end_point.z = 0;
        wall_done = true;
        wall_pts_nb++;
				single_wall.nb_inliers = wall_pts_nb;
				// ROS_INFO("wall inliers : %d", wall_pts_nb);
        add_wall_to_list();
      }
      else if((dist > maximum_space_between_points)&&(std::abs(inliers[index%tot_inliers] - inliers[(index+1)%tot_inliers]) > gap_threshold))
      {
        single_wall.end_point.x = x_ext;
        single_wall.end_point.y = y_ext;
        single_wall.end_point.z = 0;
        wall_done = true;
        wall_pts_nb++;
				single_wall.nb_inliers = wall_pts_nb;
				// ROS_INFO("wall inliers : %d", wall_pts_nb);
        add_wall_to_list();
      }
      wall_pts_nb++;
      index++;
    }
	}

	void extract_start_index()
	{
		if(x_diff > y_diff)
		{
      if(x_min_ind != 0)
      {
        if(x_min_ind == tot_inliers - 1)
        {
          index = x_max_ind;
        }
        else if(x_min_ind < x_max_ind)
        {
          index = x_max_ind;
        }
        else
        {
          index = x_min_ind;
        }
      }
      else
      {
        index = x_min_ind;
      }
    }
		else
		{
      if(y_min_ind != 0)
      {
        if(y_min_ind == tot_inliers - 1)
        {
          index = y_max_ind;
        }
        else if(y_min_ind < y_max_ind)
        {
          index = y_max_ind;
        }
        else
        {
          index = y_min_ind;
        }
      }
      else
      {
        index = y_min_ind;
      }
    }
	}

	float compute_y(float x)
	{
		return (b/a)*(x-x0) + y0;
	}

	float compute_x(float y)
	{
		return (a/b)*(y-y0) + x0;
	}

	void add_wall_to_list()
	{
		if(((single_wall.init_point.x != single_wall.end_point.x)||(single_wall.init_point.y != single_wall.end_point.y))&&(single_wall.nb_inliers >= min_point))
		{
			wall_full_list.push_back(single_wall);
      the_corners_list.push_back(single_wall.init_point);
      the_corners_list.push_back(single_wall.end_point);
			wall_nb++;
		}
	}

	void sort_datas()
	{

	}

private:
  //Input and output of the ransac sever
	robo7_msgs::wallPoint the_cloud;

  robo7_msgs::XY_coordinates point_cloud_XY_2;
  Eigen::VectorXf model_param_;

	bool still_walls; // To know if there is still some walls to find
	bool wall_done; //To know if you reach the end of the wall

  float x1, y1, x2, y2, x1_bis;
	float y_diff;
	float x_diff;

	std::vector<float> X_wall_coordinates;
	std::vector<float> Y_wall_coordinates;

	std::vector<int> inliers;

	int min_point; //minimum inliers to consider the line model as a wall
	float maximum_space_between_points; //detect if there is two different walls aligned (we do not care about space smaller than robot size)
	float dist;
  int gap_threshold;

	robo7_msgs::aWall single_wall;
	std::vector<robo7_msgs::aWall> wall_full_list;
  std::vector<geometry_msgs::Vector3> the_corners_list;

	//Model parameters Extraction
	float x0; float y0; float z0; //point on line
	float a; float b; float c; //direction of the line
	float x_ext, y_ext;
  float x_ext1, y_ext1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_init;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud;

	geometry_msgs::Twist mod;
	float ransac_threshold;

  int x_min_ind, x_max_ind, y_min_ind, y_max_ind;
  int index, index1;
  int tot_inliers;
  int wall_pts_nb;
  int wall_nb;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ransac_server");

	RansacServer ransac_;

	ros::Rate loop_rate(100);

	ROS_INFO("Ransac Algorithm to find walls running");

	ros::spin();

	return 0;
}
