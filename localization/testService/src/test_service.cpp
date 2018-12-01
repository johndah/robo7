//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>

//The messages
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>
#include <robo7_msgs/Matrix3.h>
#include <robo7_msgs/MeasureRequest.h>
#include <robo7_msgs/MeasureFeedback.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_msgs/former_position.h>
#include <robo7_msgs/the_robot_position.h>

//The services
#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/RansacWall.h>
#include <robo7_srvs/ICPAlgorithm.h>
#include <robo7_srvs/callServiceTest.h>
#include <robo7_srvs/PathFollowerSrv.h>
#include <robo7_srvs/PathFollower2.h>
#include <robo7_srvs/path_planning.h>
#include <robo7_srvs/update_map.h>
#include <robo7_srvs/GoTo.h>
#include <robo7_srvs/UpdateOccupancyGrid.h>
#include <robo7_srvs/UpdateDiscretizedMap.h>



// Control @ 10 Hz
double control_frequency = 100.0;

class test_server
{
public:
  ros::NodeHandle n;
  //Subscriber
  ros::Subscriber laser_scan;
  ros::Subscriber map_point_sub;
  ros::Subscriber position_sub;
  //Clients
  ros::ServiceClient scan_to_coord_srv;
  ros::ServiceClient ransac_srv;
  ros::ServiceClient icp_srv;
  ros::ServiceClient path_follower_srv;
  ros::ServiceClient path_follower2_srv;
  ros::ServiceClient path_planning_srv;
  ros::ServiceClient mapping_srv;
  ros::ServiceClient go_to_srv;
  ros::ServiceClient update_occupancy_grid_srv;
  ros::ServiceClient update_discretized_map_srv;
  //Server
  ros::ServiceServer to_test_service;

  test_server()
  {
    n.param<float>("/test_service/cell_size", cell_size, 0.05);

    saver_grid_initialisation();

    //The service definition
    to_test_service = n.advertiseService("/localization/test_service", &test_server::test_Sequence, this);

    //The different possible service to test and their corresponding subscribings
    scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
    laser_scan = n.subscribe("/scan", 1, &test_server::laser_scan_callBack, this);
    position_sub = n.subscribe("/localization/kalman_filter/position", 1, &test_server::robot_position_callBack, this);
    position_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &test_server::the_robot_pose_callBack, this);

    path_follower_srv = n.serviceClient<robo7_srvs::PathFollowerSrv>("/kinematics/path_follower/path_follower");
    path_follower2_srv = n.serviceClient<robo7_srvs::PathFollower2>("/kinematics/path_follower/path_follower_v2");

    ransac_srv = n.serviceClient<robo7_srvs::RansacWall>("/localization/ransac");
    icp_srv = n.serviceClient<robo7_srvs::ICPAlgorithm>("/localization/icp");
    map_point_sub = n.subscribe("/ras_maze/maze_map/walls_coord_for_icp", 1, &test_server::maze_map_callBack, this);

    path_planning_srv = n.serviceClient<robo7_srvs::path_planning>("/path_planning/path_service");

    mapping_srv = n.serviceClient<robo7_srvs::update_map>("/localization/mapping/update_map");

    go_to_srv = n.serviceClient<robo7_srvs::GoTo>("/kinematics/go_to");

    update_occupancy_grid_srv = n.serviceClient<robo7_srvs::UpdateOccupancyGrid>("/localization/mapping/update_occupancy_grid");

    update_discretized_map_srv = n.serviceClient<robo7_srvs::UpdateDiscretizedMap>("/localization/mapping/update_discretized_map");
  }

  void laser_scan_callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
      the_lidar_scan = *msg;
  }

  void robot_position_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
    robot_position = *msg;
  }

  void the_robot_pose_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;
  }

  void maze_map_callBack(const robo7_msgs::cornerList::ConstPtr &msg)
  {
    all_wall_points = *msg;
  }


  bool test_Sequence(robo7_srvs::callServiceTest::Request &req,
         robo7_srvs::callServiceTest::Response &res)
  {
    done = false;
    destination_pose = req.destination;

    ROS_INFO("%d, %d", (int)-9.2, (int)9.8);

    //Plot the lidar scan in the map frame
    if(req.which_service == 0)
    {
      robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = robot_position;
      req1.lidar_scan = the_lidar_scan;
      scan_to_coord_srv.call(req1, res1);
      done = res1.success;
    }

    //Follow a published path -> need to disappear (see 5)
    else if(req.which_service == 1)
    {
      robo7_srvs::PathFollowerSrv::Request req1;
      robo7_srvs::PathFollowerSrv::Response res1;
      req1.req = true;
      path_follower_srv.call(req1, res1);
      done = res1.success;
    }

    //Extract the walls in the map frame
    else if(req.which_service == 2)
    {
      robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = robot_position;
      req1.lidar_scan = the_lidar_scan;
      scan_to_coord_srv.call(req1, res1);

      robo7_srvs::RansacWall::Request req2;
      robo7_srvs::RansacWall::Response res2;
      req2.the_cloud = res1.the_lidar_points;
      ransac_srv.call(req2, res2);
      done = true;
    }

    //Apply the convergence icp with the extracted corners of the ransac algorithm
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
      req3.current_position = robot_position;
      req3.the_lidar_corners = res2.all_corners;
      req3.the_wall_corners = all_wall_points;
      icp_srv.call(req3, res3);
    }

    //Make the convergence between the lidar coordinates and the discretize wall
    else if(req.which_service == 4)
    {
      robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = robot_position;
      req1.lidar_scan = the_lidar_scan;
      scan_to_coord_srv.call(req1, res1);
      done = res1.success;

      robo7_srvs::ICPAlgorithm::Request req3;
      robo7_srvs::ICPAlgorithm::Response res3;
      req3.current_position = robot_position;
      req3.the_lidar_corners = res1.the_lidar_point_cloud;
      req3.the_wall_corners = all_wall_points;
      icp_srv.call(req3, res3);
    }

    //Follow a path precomputed by the path_planning service
    else if(req.which_service == 5)
    {
      robo7_srvs::path_planning::Request req1;
      robo7_srvs::path_planning::Response res1;
      req1.robot_position = robot_position;
      req1.destination_position = destination_pose;
      path_planning_srv.call(req1, res1);

      robo7_srvs::PathFollower2::Request req2;
      robo7_srvs::PathFollower2::Response res2;
      req2.trajectory = res1.path;
      req2.traject = res1.path_planned;
      path_follower2_srv.call(req2, res2);
      done = res1.success;
    }

    //Move the robot to another position
    else if(req.which_service == 6)
    {
      // ROS_INFO("Start service go to");
      // robo7_srvs::GoTo::Request req1;
      // robo7_srvs::GoTo::Response res1;
      // req1.robot_pose = robot_position;
      // req1.destination_pose = destination_pose;
      // go_to_srv.call(req1, res1);
      // done = res1.success;
    }

    //Test service for the update occupancy grid
    else if(req.which_service == 7)
    {
      robo7_srvs::UpdateOccupancyGrid::Request req1;
      robo7_srvs::UpdateOccupancyGrid::Response res1;
      req1.the_robot_pose = the_robot_pose;
      req1.occupancy_grid = the_grid_saved;
      update_occupancy_grid_srv.call(req1, res1);
      the_grid_saved = res1.updated_occupancy_grid;
      done = res1.success;
    }

    //Test service for the update discretized map
    else if(req.which_service == 8)
    {
      robo7_srvs::UpdateDiscretizedMap::Request req1;
      robo7_srvs::UpdateDiscretizedMap::Response res1;
      req1.occupancy_grid = the_grid_saved;
      update_discretized_map_srv.call(req1, res1);
      the_walls_discretized = res1.discretized_walls;
    }


    res.success = done;
    return true;
  }


private:
  sensor_msgs::LaserScan the_lidar_scan;
  geometry_msgs::Twist robot_position;
  geometry_msgs::Point destination_pose;

  robo7_msgs::cornerList all_wall_points;

  bool done;

  robo7_msgs::wallList map_walls;
  robo7_msgs::former_position robot_pos;
  robo7_msgs::the_robot_position the_robot_pose;

  robo7_msgs::mapping_grid the_grid_saved;
  robo7_msgs::matrix initial_grid;
  robo7_msgs::matrix_row one_row;
  float cell_size;

  robo7_msgs::wallPoint the_walls_discretized;

  void saver_grid_initialisation()
  {
    the_grid_saved.header.seq = 0;
    the_grid_saved.header.stamp = ros::Time::now();
    the_grid_saved.window_width = cell_size;
    the_grid_saved.window_height = cell_size;
    the_grid_saved.cell_size = cell_size;
    the_grid_saved.top_left_corner.x = cell_size * (int)(the_robot_pose.position.linear.x/cell_size);
    the_grid_saved.top_left_corner.y = cell_size * (int)(the_robot_pose.position.linear.y/cell_size);
    one_row.cols.clear();
    one_row.cols.push_back(0);
    initial_grid.nb_rows = 1;
    initial_grid.nb_cols = 1;
    initial_grid.rows.push_back(one_row);
    the_grid_saved.occupancy_grid = initial_grid;
  }

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
