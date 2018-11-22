// #include <algorithm>
#include <stdlib.h>     /* abs */
#include <ros/ros.h>
//Messages
#include <geometry_msgs/Twist.h>
#include <robo7_msgs/destination_point.h>
#include <robo7_msgs/trajectory.h>
#include <geometry_msgs/Point.h>

//Services
#include <robo7_srvs/PathFollowerSrv.h>
#include <robo7_srvs/path_planning.h>
#include <robo7_srvs/GoTo.h>

class go_to
{
public:
  ros::NodeHandle n;
  //Client
  ros::ServiceClient path_planning_srv;
  ros::ServiceClient path_follower_srv;
  //Server
  ros::ServiceServer go_to_server;

  go_to()
  {
    //Service client
    path_planning_srv = n.serviceClient<robo7_srvs::path_planning>("/path_planning/path_service");
    path_follower_srv = n.serviceClient<robo7_srvs::PathFollowerSrv>("/kinematics/path_follower/path_follower");

    //Server
    go_to_server = n.advertiseService("/kinematics/go_to", &go_to::goToSequence, this);
  }

  bool goToSequence(robo7_srvs::GoTo::Request &req,
         robo7_srvs::GoTo::Response &res)
  {
    robot_pose = req.robot_pose;
    destination_pose = req.destination_pose;
    
    //Convert
    change_twist_to_point();

    //Find the path_planned
    robo7_srvs::path_planning::Request req1;
    robo7_srvs::path_planning::Response res1;
    req1.robot_position = robot_pose;
    req1.destination_position = destination_position;
    path_planning_srv.call(req1, res1);

    //Find the path_planned
    robo7_srvs::PathFollowerSrv::Request req2;
    robo7_srvs::PathFollowerSrv::Response res2;
    req2.req = true;
    req2.trajectory = res1.path_planned;
    path_follower_srv.call(req2, res2);

    res.success = res2.success;
  }


private:
  geometry_msgs::Twist robot_pose;
  geometry_msgs::Twist destination_pose;

  geometry_msgs::Point destination_position;

  void change_twist_to_point()
  {
    destination_position.x = destination_pose.linear.x;
    destination_position.y = destination_pose.linear.y;
    destination_position.z = destination_pose.linear.z;
  }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to");

    go_to go_to_;

    ROS_INFO("Go To running");

    ros::spin();

    return 0;
}
