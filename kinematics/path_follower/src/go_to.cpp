// #include <algorithm>
#include <stdlib.h>     /* abs */
#include <ros/ros.h>
//Messages
#include <geometry_msgs/Twist.h>
#include <robo7_msgs/the_robot_position.h>
#include <robo7_msgs/trajectory.h>
#include <geometry_msgs/Point.h>

//Services
#include <robo7_srvs/PathFollower2.h>
#include <robo7_srvs/path_planning.h>
#include <robo7_srvs/GoTo.h>
#include <robo7_srvs/PureRotation.h>

class go_to
{
public:
  ros::NodeHandle n;
  //Subscriber
  ros::Subscriber robot_position_sub;
  //Client
  ros::ServiceClient path_planning_srv;
  ros::ServiceClient path_follower2_srv;
  ros::ServiceClient pure_rotation_srv;
  //Server
  ros::ServiceServer go_to_server;

  go_to()
  {
    //Service client
    path_planning_srv = n.serviceClient<robo7_srvs::path_planning>("/path_planning/path_service");
    path_follower2_srv = n.serviceClient<robo7_srvs::PathFollower2>("/kinematics/path_follower/path_follower_v2");
    pure_rotation_srv = n.serviceClient<robo7_srvs::PureRotation>("/kinematics/path_follower/pure_rotation");

    //Subscriber
    robot_position_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &go_to::position_callBack, this);

    //Server
    go_to_server = n.advertiseService("/kinematics/go_to", &go_to::goToSequence, this);
  }

  void position_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;
  }

  bool goToSequence(robo7_srvs::GoTo::Request &req,
         robo7_srvs::GoTo::Response &res)
  {
    destination_pose = req.destination_pose;
    bool arrived = false;

    //Convert
    change_twist_to_point();

    while(!arrived)
    {
      ros::spinOnce();
      //Find the path_planned
      robo7_srvs::path_planning::Request req1;
      robo7_srvs::path_planning::Response res1;
      // ROS_INFO("(x,y,theta) = (%lf, %lf, %lf)", the_robot_pose.position.linear.x, the_robot_pose.position.linear.y, the_robot_pose.position.angular.z);
      req1.robot_position = the_robot_pose.position;
      req1.destination_position = destination_position;
      path_planning_srv.call(req1, res1);

      //Find the path_planned
      robo7_srvs::PathFollower2::Request req2;
      robo7_srvs::PathFollower2::Response res2;
      req2.traject = res1.path_planned;
      path_follower2_srv.call(req2, res2);
      arrived = res2.success;
    }


    //Turn on itself to the right direction
    if(destination_pose.angular.x != -1)
    {
      robo7_srvs::PureRotation::Request req3;
      robo7_srvs::PureRotation::Response res3;
      req3.desired_angle = destination_pose.angular.z;
      pure_rotation_srv.call(req3, res3);
    }

    res.success = arrived;
  }


private:
  robo7_msgs::the_robot_position the_robot_pose;
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
