// #include <algorithm>
#include <stdlib.h>     /* abs */
#include <ros/ros.h>
//Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <robo7_msgs/destination_point.h>
#include <robo7_msgs/trajectory.h>
//Services
#include <robo7_srvs/PathFollower2.h>
#include <robo7_srvs/PureRotation.h>

float control_frequency = 10.0;
float pi = 3.14;

class path_follower_v2
{
public:
  ros::NodeHandle n;
  ros::Subscriber robot_pose_sub;
  ros::Publisher desired_velocity_pub;
  ros::ServiceServer path_follower_server;
  ros::ServiceClient pure_rotation_srv;

  path_follower_v2()
  {
    //Initialisation parameters
    n.param<float>("/path_follower_v2/distance_to_destination_threshold", dest_threshold, 0.01);
    n.param<float>("/path_follower_v2/distance_to_jump_to_next_point", dest_to_next_point, 0.05);
    n.param<float>("/path_follower_v2/angle_P", a_P, 0.05);
    n.param<float>("/path_follower_v2/angular_velocity_saturation", desire_angular_sat, 0.05);
    n.param<float>("/path_follower_v2/angular_threshold_trust", angle_ref_max, pi/8);
    n.param<float>("/path_follower_v2/angular_threshold_trust", aver_lin_vel, 0.0);

    current_point_to_follow = 0;

    robot_pose_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &path_follower::position_callBack, this);

    desired_velocity_pub = n.advertise<geometry_msgs::Twist>("/desired_velocity", 1);

    pure_rotation_srv = n.serviceClient<robo7_srvs::PureRotation>("/kinematics/path_follower/pure_rotation");

    path_follower_server = n.advertiseService("/kinematics/path_follower/path_follower_v2", &path_follower::path_follower_Sequence, this);
  }

  void position_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;
  }

  bool path_follower_Sequence(robo7_srvs::PathFollower2::Request &req,
         robo7_srvs::PathFollower2::Response &res)
  {
    robo7_msgs::target_trajectory the_trajectory = req.trajectory;

    bool path_ended = false;

    robo7_msgs::wallPoint the_discretized_path = discretize_the_path( the_trajectory , discretize_length );

    if(point_follower_mode)
    {
      //First align the robot with the path start (aka call service)
      robo7_srvs::PureRotation::Request req1;
      robo7_srvs::PureRotation::Response res1;
      req1.desired_angle = initial_angle;
      pure_rotation_srv.call(req1, res1);

      int index_point_following = 0;
      //Then make it follow the path
      while(!path_ended)
      {
        //Extract the position with the subscriber
        ros::spinOnce();

        float velocity_sign = 1;

        //Choose the point to follow
        int index_point_following = point_to_follow( the_discretized_path , index_point_following );
        geometry_msgs::Vector3 point_following = the_discretized_path.the_points[index_point_following];

        //Extract the angle difference out of this point to follow
        diff_angle = compute_diff_angle( point_following );

        //Then update the linear speed
        if(sgn(diff_angle)*diff_angle > angle_ref_max)
        {
          desire_vel.linear.x = 0;
          desire_vel.angular.z = P_update( diff_angle );
          angle_ref_max = pi/8;
        }
        else
        {
          desire_vel.linear.x = velocity_sign * aver_lin_vel;
          desire_vel.angular.z = P_update( diff_angle );
          angle_ref_max = pi/3;
        }

        //Then check if the path came to an end
        if((index_point_following == the_discretized_path.number-1)
            &&(distance_to_point( the_discretized_path.the_points[index_point_following]) < dest_threshold))
          {
            path_ended = true;
            desire_vel.linear.x = 0;
            desire_vel.angular.z = 0;
          }

        //Then publish the speed
        desired_velocity_pub.publish( desire_vel );
      }

      //Republish an 0 speed
      desire_vel.linear.x = 0;
      desire_vel.angular.z = 0;
      desired_velocity_pub.publish( desire_vel );
    }

    res.success = true;
  }


private:
  //Subscribers
  robo7_msgs::the_robot_position the_robot_pose;

  //Controllers values
  float a_p;
  float desire_angular_sat;
  float angle_ref_max;
  float aver_lin_vel;

  //Which path follower we choose
  bool point_follower_mode;

  void compute_distance_to_current_destination()
  {
    distance_to_destination = sqrt(pow(robot_x-trajectory_array.trajectory_points[current_point_to_follow].point_coord.x, 2) + pow(robot_y-trajectory_array.trajectory_points[current_point_to_follow].point_coord.y, 2));
  }

  void update_Destination_Point()
  {

  }

  int point_to_follow( robo7_msgs::wallPoint discretized_path_msg, int current_index)
  {
    int the_new_index = current_index;

    if(distance_to_point(discretized_path_msg.the_points[the_new_index]) < dest_to_next_point)
    {
      while((distance_to_point(discretized_path_msg.the_points[the_new_index]) < dest_to_next_point)
              &&(the_new_index < discretized_path_msg.number))
        {
          the_new_index++;
        }
    }

    return the_new_index;
  }

  float distance_to_point( geometry_msgs::Vector3 the_point )
  {
    float x_r = the_robot_pose.position.linear.x;
    float y_r = the_robot_pose.position.linear.y;

    return sqrt(pow(x_r-the_point.x,2)+pow(y_r-the_point.y,2));
  }

  float P_update(float difference_angle)
  {
    error = difference_angle;

    float angular_vel = (a_P * error);
    float angular_sat = desire_angular_sat;

    // ROS_INFO("angular vel = %lf and %lf", angular_vel, angular_sat);

    if(angular_vel > angular_sat) { angular_vel = angular_sat; }
    else if(-angular_vel < -angular_sat) { angular_vel = -angular_sat; }

    return angular_vel;
  }

  float compute_diff_angle(geometry_msgs::Vector3 to_follow)
  {
    float x_r = the_robot_pose.position.linear.x;
    float y_r = the_robot_pose.position.linear.y;
    float a_r = the_robot_pose.position.angular.z;

    float x = (to_follow.x - x_r) * cos(a_r) + (to_follow.y - y_r) * sin(a_r);
    float y = - (to_follow.x - x_r) * sin(a_r) + (to_follow.y - y_r) * cos(a_r);

    return findangle(x, y);
  }

  robo7_msgs::wallPoint discretize_the_path( robo7_msgs::target_trajectory trajectory , float l )
  {
    robo7_msgs::wallPoint discretized_path;

    for(int i=0; i<trajectory.number_of_path; i++)
    {
      geometry_msgs::Vector3 aPoint;
      aPoint.x = trajectory.target_trajectory_points[i].end_point.linear.x;
      aPoint.y = trajectory.target_trajectory_points[i].end_point.linear.y;
      if(trajectory.target_trajectory_points[i].is_it_line)
      {
        aPoint.z = 0.2; //Distance before switching point
      }
      else
      {
        aPoint.z = 0.1; //Distance before switching for curves
      }
    }

    return discretized_path;
  }

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower_v2");

    path_follower_v2 path_follower_v2_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Path Follower v2 running");

    ros::spin();

    return 0;
}
