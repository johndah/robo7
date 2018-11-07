// #include <algorithm>
#include <stdlib.h>     /* abs */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robo7_msgs/destination_point.h>
#include <robo7_msgs/trajectory.h>
#include <robo7_srvs/PathFollowerSrv.h>

float control_frequency = 10.0;

class path_follower
{
public:
  ros::NodeHandle n;
  ros::Subscriber trajectory_sub;
  ros::Subscriber robot_position;
  ros::Publisher destination_pub;
  ros::ServiceServer path_follower_service;

  path_follower()
  {
    n.param<float>("/path_follower/distance_to_destination_threshold", dest_threshold, 0.01);
    n.param<float>("/path_follower/distance_to_jump_to_next_point", dest_to_next_point, 0.05);

    current_point_to_follow = 0;

    trajectory_sub = n.subscribe("/path_planning/trajectory", 1, &path_follower::trajectory_callBack, this);
    robot_position = n.subscribe("/localization/kalman_filter/position", 1, &path_follower::position_callBack, this);
    destination_pub = n.advertise<robo7_msgs::destination_point>("/kinematics/path_follower/dest_point", 1);

    path_follower_service = n.advertiseService("/kinematics/path_follower/path_follower", &path_follower::path_follower_Sequence, this);
  }

  void position_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
    robot_x = msg->linear.x;
    robot_y = msg->linear.y;
    robot_theta = msg->angular.z;
  }

  void trajectory_callBack(const robo7_msgs::trajectory::ConstPtr &msg)
  {
    // if(trajectory.number != 0)&&(trajectory.id_number != trajectory_array.id_number)
    // {
      // trajectory_array = *msg;
      // current_point_to_follow = 0;
    // }
    // ROS_INFO("%ld", static_cast<int>(trajectory_array.trajectory_points.size()));
    // if( static_cast<int>(trajectory_array.trajectory_points.size()) > 0)
    // {
    //   ROS_INFO("%lf, %lf", trajectory_array.trajectory_points[0].point_coord.x,  trajectory_array.trajectory_points[0].point_coord.y);
    // }

  }

  void update_Destination_Point()
  {
    ros::spinOnce();

    compute_distance_to_current_destination();

    // ROS_INFO("Distance to current point : %lf", distance_to_destination);
    // ROS_INFO("Point to follow : %lf, %lf", trajectory_array.trajectory_points[current_point_to_follow].point_coord.x,  trajectory_array.trajectory_points[current_point_to_follow].point_coord.y);

    if(static_cast<int>(trajectory_array.trajectory_points.size()) > 0)
    {
      // ROS_INFO("New_path");
      // ROS_INFO("distance_to_dest : %lf, distance_next_point : %lf", distance_to_destination,trajectory_array.trajectory_points[current_point_to_follow].distance);
      // ROS_INFO("Robot x,y : %lf, %lf", robot_x, robot_y);
      // (distance_to_destination < trajectory_array.trajectory_points[current_point_to_follow].distance)||
      if(((distance_to_destination < dest_to_next_point))&&(current_point_to_follow < static_cast<int>(trajectory_array.trajectory_points.size()) - 1))
      {
        // ROS_INFO("Next_point");
        current_point_to_follow += 1;
      }
      update_the_destination();
    }

    destination_pub.publish( the_destination );
  }

  bool path_follower_Sequence(robo7_srvs::PathFollowerSrv::Request &req,
         robo7_srvs::PathFollowerSrv::Response &res)
  {
    trajectory_array = req.trajectory;

    ROS_INFO("Start to follow the path");
    arrived = false;
    current_point_to_follow = 0;
    while(!arrived)
    {
      // ROS_INFO("Not arrived");
      update_Destination_Point();
      // ROS_INFO("Point to follow : %d, (x,y) = (%lf, %lf)", current_point_to_follow, trajectory_array.trajectory_points[current_point_to_follow].point_coord.x, trajectory_array.trajectory_points[current_point_to_follow].point_coord.y);
      if(((distance_to_destination < dest_threshold)&&(current_point_to_follow == static_cast<int>(trajectory_array.trajectory_points.size()) - 1)))
      {
        ROS_INFO("Arrived");
        arrived = true;
      }
    }

    res.success = true;
  }


private:
  geometry_msgs::Twist dest_twist;
  geometry_msgs::Twist desire_vel;
  robo7_msgs::trajectory trajectory_array;
  robo7_msgs::destination_point the_destination;

  //robot position
  float robot_x;
  float robot_y;
  float robot_theta;

  //Point destination data
  int current_point_to_follow;
  float distance_to_destination;
  float dest_to_next_point;
  float dest_threshold;
  bool arrived;

  void compute_distance_to_current_destination()
  {
    distance_to_destination = sqrt(pow(robot_x-trajectory_array.trajectory_points[current_point_to_follow].point_coord.x, 2) + pow(robot_y-trajectory_array.trajectory_points[current_point_to_follow].point_coord.y, 2));
  }

  void update_the_destination()
  {
    the_destination.destination.linear.x = trajectory_array.trajectory_points[current_point_to_follow].point_coord.x;
    the_destination.destination.linear.y = trajectory_array.trajectory_points[current_point_to_follow].point_coord.y;
    the_destination.destination.linear.z = trajectory_array.trajectory_points[current_point_to_follow].point_coord.z;

    the_destination.speed = trajectory_array.trajectory_points[current_point_to_follow].speed;
  }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower");

    path_follower path_follower_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Path Follower running");

    while(path_follower_.n.ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
