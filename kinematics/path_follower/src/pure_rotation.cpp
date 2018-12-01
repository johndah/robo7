// #include <algorithm>
#include <stdlib.h>     /* abs */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robo7_msgs/destination_point.h>
#include <robo7_msgs/the_robot_position.h>
#include <robo7_srvs/PureRotation.h>

float control_frequency = 50.0;
float pi = 3.14;
float dt = 1/control_frequency;

class pure_rotation
{
public:
  ros::NodeHandle n;
  ros::Subscriber robot_position;
  ros::Publisher desired_velocity;
  ros::ServiceServer pure_rotation_service;

  pure_rotation()
  {
    //Initialisation
    n.param<float>("/pure_rotation/margins_acceptance", angle_deviation_acceptance, pi/8);
    n.param<float>("/pure_rotation/angle_P", a_P, 0.0);
    n.param<float>("/pure_rotation/angular_velocity_saturation_threshold", desire_angular_sat, 0.0);

    robot_position = n.subscribe("/localization/kalman_filter/position_timed", 1, &pure_rotation::position_callBack, this);
    desired_velocity = n.advertise<geometry_msgs::Twist>("/desired_velocity", 1);

    pure_rotation_service = n.advertiseService("/kinematics/path_follower/pure_rotation", &pure_rotation::rotation_Sequence, this);
  }

  void position_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;
  }

  bool rotation_Sequence(robo7_srvs::PureRotation::Request &req,
         robo7_srvs::PureRotation::Response &res)
  {
    //Pick back the desired angle
    float desire_angle = wrapAngle(req.desired_angle);
    geometry_msgs::Twist desire_vel;

    ros::Rate loop_rate(control_frequency);

    ros::spinOnce();
    float diff_angle = desire_angle - the_robot_pose.position.angular.z;

    while(sgn(diff_angle)*diff_angle > angle_deviation_acceptance)
    {
      ros::spinOnce();
      diff_angle = desire_angle - the_robot_pose.position.angular.z;

      // ROS_INFO("diff_angle = %lf", diff_angle);

      if(sgn(diff_angle)*diff_angle > pi)
      {
        if(sgn(diff_angle) > 0)
        {
          diff_angle = diff_angle - 2*pi;
        }
        else
        {
          diff_angle = 2*pi + diff_angle;
        }
      }

      // ROS_INFO("diff_angle = %lf", diff_angle);

      desire_vel.linear.x = 0;
      desire_vel.angular.z = P_update( diff_angle );

      // ROS_INFO("desire_vel = %lf", desire_vel.angular.z);

      desired_velocity.publish( desire_vel );
      loop_rate.sleep();
    }

    //Then stop the rotation
    desire_vel.linear.x = 0;
    desire_vel.angular.z = 0;
    desired_velocity.publish( desire_vel );

    res.success = true;
  }


private:
  robo7_msgs::the_robot_position the_robot_pose;

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

  //Corrector Values
  //Controller parameters
  float a_P;
  float a_I;
  float a_D;
  float angle_deviation_acceptance;
  float dist_left;

  float error;
  float anti_windup;
  float dif_error;
  float int_error;
  float err_sat;
  float desire_angular_vel;
  float desire_angular_sat;
  float desire_vel_threshold;

  void PID_update( float diff_angle )
  {
    dif_error = diff_angle - error;

    error = diff_angle;

    int_error = int_error + error * dt;

    if(int_error > err_sat) { int_error = err_sat; }
    else if(int_error < -err_sat) { int_error = -err_sat; }

    desire_angular_vel = (a_P * error + a_I * int_error + a_D * dif_error / dt);
  }

  void PID_AWU_update( float diff_angle )
  {
    dif_error = diff_angle - error;

    error = diff_angle;

    int_error = int_error + ( error + anti_windup ) * dt;

    desire_angular_vel = (a_P * error + a_I * int_error);
    desire_angular_sat = desire_angular_vel;

    if(desire_angular_sat > desire_vel_threshold) { desire_angular_sat = desire_vel_threshold; }
    else if(desire_angular_sat < -desire_vel_threshold) { desire_angular_sat = -desire_vel_threshold; }

    anti_windup = ( desire_angular_sat - desire_angular_vel ) / dt;

    desire_angular_vel = desire_angular_sat;
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

  void initialisation_of_parameters()
  {
    error = 0;
    anti_windup = 0;
    dif_error = 0;
    int_error = 0;
    err_sat = 0;
    desire_angular_vel = 0;
    desire_angular_sat = 0;
    desire_vel_threshold = 0;
  }

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_rotation");

    pure_rotation pure_rotation_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Pure Rotation running");

    ros::spin();

    return 0;
}
