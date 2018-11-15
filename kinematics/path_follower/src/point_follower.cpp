#include <algorithm>
#include <stdlib.h>     /* abs */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "robo7_msgs/WheelAngularVelocities.h"
#include "robo7_msgs/destination_point.h"
#include "robo7_msgs/the_robot_position.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

robo7_msgs::destination_point dest_twist;
robo7_msgs::the_robot_position robot_position;
geometry_msgs::Twist desire_vel;

float freq = 100;
float dt = 1/freq;
float pi = 3.14159;
int break_scalar = 5;

double msg_timeout = 0.004; // acepted delay between msgs before stopping

//Destination parameters
float x_point = 0.215;
float y_point = 0.2;
bool new_point;

// From rosparam get /robot_description/
double wheel_separation = 0.2198; // center of track to center of track
double wheel_radius = 0.049;
double max_allowed_speed = 25; //rad.s-1

//robot position
float robot_x;
float robot_y;
float robot_theta;

// Average velocity of the robot
float aver_lin_vel;
float velocity_sign;
float x_point_robot;
float y_point_robot;

//Break info parameters
bool danger;
bool drive_backward;

//Controller parameters
float a_P;
float a_I;
float a_D;
float dist_left;
float diff_angle;
float dist_threshold = 0.05;
float angle_ref_max = pi/8;

float error = 0;
float anti_windup = 0;
float dif_error = 0;
float int_error = 0;
float err_sat = 0;
float desire_angular_vel = 0;
float desire_angular_sat = 0;
float desire_vel_threshold = 0;

bool arrived;
bool problem = false;
bool new_measure;

double duration;


void destination_callback(const robo7_msgs::destination_point::ConstPtr &msg)
{
  dest_twist = *msg;
  arrived = false;
  // if((x_point != dest_twist.linear.x)||(y_point != dest_twist.linear.y))
  // {
  //   new_point = true;
  // }
  // else
  // {
  //   new_point = false;
  // }
  x_point = dest_twist.destination.linear.x;
  y_point = dest_twist.destination.linear.y;
}

void position_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
{
  if((msg->header.seq != robot_position.header.seq)&&(msg->header.seq > 1))
  {
    robot_position = *msg;
    robot_x = robot_position.position.linear.x;
    robot_y = robot_position.position.linear.y;
    robot_theta = robot_position.position.angular.z;
    new_measure = true;
  }
}

void break_callBack(const std_msgs::Bool::ConstPtr &msg)
{
  danger = msg->data;
  if(danger)
  {
    problem = true;
  }
}

int sgn(float v)
{
  if (v < 0) return -1;
  else if (v > 0) return 1;
  else return 0;
}

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

float wrapAngle( double angle )
{
  float twoPi = 2.0 * pi;
  return angle - twoPi * floor( angle / twoPi );
}

void PID_update()
{
  dif_error = diff_angle - error;

  error = diff_angle;

  int_error = int_error + error * dt;

  if(int_error > err_sat) { int_error = err_sat; }
  else if(int_error < -err_sat) { int_error = -err_sat; }

  desire_angular_vel = (a_P * error + a_I * int_error + a_D * dif_error / dt);
}

void PID_AWU_update()
{
  dif_error = diff_angle - error;

  error = diff_angle;

  int_error = int_error + ( error + anti_windup ) * dt;

  desire_angular_vel = (a_P * error + a_I * int_error + a_D * dif_error / dt);
  desire_angular_sat = desire_angular_vel;

  if(desire_angular_sat > desire_vel_threshold) { desire_angular_sat = desire_vel_threshold; }
  else if(desire_angular_sat < -desire_vel_threshold) { desire_angular_sat = -desire_vel_threshold; }

  anti_windup = ( desire_angular_sat - desire_angular_vel ) / dt;

  desire_angular_vel = desire_angular_sat;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_follower");

  ros::NodeHandle n("~");

  n.param<float>("/point_follower/angle_P", a_P, 0);
  n.param<float>("/point_follower/angle_I", a_I, 0);
  n.param<float>("/point_follower/angle_D", a_D, 0);
  n.param<float>("/point_follower/error_sat", err_sat, 10);
  n.param<float>("/point_follower/angular_velocity_saturation_threshold", desire_vel_threshold, 10);
  n.param<float>("/point_follower/linear_speed", aver_lin_vel, 0);
  n.param<bool>("/point_follower/drive_backward", drive_backward, false);

  ros::Subscriber twist_sub = n.subscribe("/kinematics/path_follower/dest_point", 1, destination_callback);
  ros::Subscriber robot_position = n.subscribe("/localization/kalman_filter/position_timed", 1, position_callBack);
  ros::Subscriber breaker = n.subscribe("/break_info", 1, break_callBack);
  ros::Publisher desired_velocity = n.advertise<geometry_msgs::Twist>("/desired_velocity", 1);
  ros::Publisher dest_point = n.advertise<geometry_msgs::Twist>("/point_destination_robot", 1);
  ros::Publisher robot_arrived = n.advertise<std_msgs::Bool>("/robot_arrived", 1);

  ros::Publisher integ = n.advertise<std_msgs::Float32>("/integ", 1);

  ros::Rate loop_rate(freq);

  ROS_INFO("Running point_follower");

  std_msgs::Bool is_robot_arrived;
  geometry_msgs::Twist help_msg;
  geometry_msgs::Twist point_plot;
  std_msgs::Float32 integ_err;

  while (ros::ok())
  {
    ros::spinOnce();

    //Transforming the point in the robot frame
    x_point_robot = (x_point - robot_x) * cos(robot_theta) + (y_point - robot_y) * sin(robot_theta);
    y_point_robot = - (x_point - robot_x) * sin(robot_theta) + (y_point - robot_y) * cos(robot_theta);
    point_plot.linear.x = x_point_robot;
    point_plot.linear.y = y_point_robot;

    dist_left = sqrt(pow(x_point_robot,2) + pow(y_point_robot,2));
    if(!drive_backward)
    {
      diff_angle = findangle(x_point_robot, y_point_robot);
      velocity_sign = +1;
    }
    else
    {
      diff_angle = findangle(x_point_robot, y_point_robot);
      diff_angle = wrapAngle(diff_angle) - pi;
      velocity_sign = -1;
    }


    if(dist_left < dist_threshold)
    {
        arrived = true;
        desire_vel.linear.x = 0;
        desire_vel.angular.z = 0;
    }


    if((!arrived)&&(!problem)&&(new_measure))
    {
      if(sgn(diff_angle)*diff_angle > angle_ref_max)
      {
        desire_vel.linear.x = 0;
        PID_AWU_update();
        desire_vel.angular.z = desire_angular_vel;
      }
      else
      {
        desire_vel.linear.x = velocity_sign * aver_lin_vel;
        PID_AWU_update();
        desire_vel.angular.z = desire_angular_vel;
      }
      integ_err.data = int_error;
      integ.publish( integ_err );
      new_measure = false;
    }
    else if(problem)
    {
      desire_vel.linear.x = 0;
      desire_vel.angular.z = 0;
    }

    if(dist_left > dist_threshold)
    {
      arrived = false;
    }

    is_robot_arrived.data = arrived;

    robot_arrived.publish( is_robot_arrived );
    dest_point.publish( point_plot);
    desired_velocity.publish(desire_vel);
    loop_rate.sleep();
  }

  return 0;
}
