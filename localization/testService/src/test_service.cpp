//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>
#include <robo7_msgs/Matrix3.h>
#include <robo7_msgs/MeasureRequest.h>
#include <robo7_msgs/MeasureFeedback.h>
#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/RansacWall.h>
#include <robo7_srvs/ICPAlgorithm.h>
#include <robo7_srvs/callServiceTest.h>
#include <robo7_srvs/PathFollowerSrv.h>



// Control @ 10 Hz
double control_frequency = 100.0;

class test_server
{
public:
  ros::NodeHandle n;
  ros::Subscriber laser_scan;
  ros::ServiceClient scan_to_coord_srv;
  ros::ServiceClient ransac_srv;
  ros::ServiceClient icp_srv;
  ros::ServiceClient path_follower_srv;
  ros::ServiceServer to_test_service;

  test_server()
  {
    //The service definition
    to_test_service = n.advertiseService("/localization/test_service", &test_server::test_Sequence, this);

    //The different possible service to test and their corresponding subscribings
    scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
    laser_scan = n.subscribe("/scan", 1, &test_server::laser_scan_callBack, this);
    laser_scan = n.subscribe("/localization/kalman_filter/position", 1, &test_server::robot_position_callBack, this);

    path_follower_srv = n.serviceClient<robo7_srvs::PathFollowerSrv>("/kinematics/path_follower/path_follower");

    ransac_srv = n.serviceClient<robo7_srvs::RansacWall>("/localization/ransac");
    icp_srv = n.serviceClient<robo7_srvs::ICPAlgorithm>("/localization/icp");
  }

  void laser_scan_callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
      the_lidar_scan.header = msg->header;
      the_lidar_scan.angle_min = msg->angle_min;
      the_lidar_scan.angle_max = msg->angle_max;
      the_lidar_scan.angle_increment = msg->angle_increment;
      the_lidar_scan.time_increment = msg->time_increment;
      the_lidar_scan.scan_time = msg->scan_time;
      the_lidar_scan.range_min = msg->range_min;
      the_lidar_scan.range_max = msg->range_max;
      the_lidar_scan.ranges = msg->ranges;
      the_lidar_scan.intensities = msg->intensities;
  }

  void robot_position_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
    robot_position = *msg;
  }


  bool test_Sequence(robo7_srvs::callServiceTest::Request &req,
         robo7_srvs::callServiceTest::Response &res)
  {
    done = false;

    if(req.which_service == 0)
    {
      robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = robot_position;
      req1.lidar_scan = the_lidar_scan;
      scan_to_coord_srv.call(req1, res1);
      done = res1.success;
    }

    else if(req.which_service == 1)
    {
      robo7_srvs::PathFollowerSrv::Request req1;
      robo7_srvs::PathFollowerSrv::Response res1;
      req1.req = true;
      path_follower_srv.call(req1, res1);
      done = res1.success;
    }

    else if(req.which_service == 2)
    {
      robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = robot_position;
      req1.lidar_scan = the_lidar_scan;
      scan_to_coord_srv.call(req1, res1);
      done = res1.success;

      robo7_srvs::RansacWall::Request req2;
      robo7_srvs::RansacWall::Response res2;
      req2.point_cloud = res1.point_cloud_coordinates;
      ransac_srv.call(req2, res2);
    }

    else if(req.which_service == 3)
    {
      robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = robot_position;
      req1.lidar_scan = the_lidar_scan;
      scan_to_coord_srv.call(req1, res1);
      done = res1.success;

      robo7_srvs::RansacWall::Request req2;
      robo7_srvs::RansacWall::Response res2;
      req2.point_cloud = res1.point_cloud_coordinates;
      ransac_srv.call(req2, res2);

      robo7_srvs::ICPAlgorithm::Request req3;
      robo7_srvs::ICPAlgorithm::Response res3;
      req3.the_lidar_corners = res2.all_corners;
      icp_srv.call(req3, res3);

    }

    res.success = done;

    return true;
  }


private:
  sensor_msgs::LaserScan the_lidar_scan;
  geometry_msgs::Twist robot_position;

  bool done;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_server");

	test_server test_server_;

	ros::Rate loop_rate(100);

	ROS_INFO("The test_service's Service is ready");

	ros::spin();

	return 0;
}
