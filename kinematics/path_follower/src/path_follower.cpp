#include <algorithm>
#include <stdlib.h>     /* abs */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "robo7_msgs/WheelAngularVelocities.h"
#include "std_msgs/Bool.h"

geometry_msgs::Twist dest_twist;
geometry_msgs::Twist pos_twist;
geometry_msgs::Twist desire_vel;

int freq = 10;
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
float x_point_robot;
float y_point_robot;

//Break info parameters
bool danger;

//Controller parameters
float P = 1;
float dist_left;
float diff_angle;
float dist_threshold = 0.05;
float angle_ref_max = pi/8;

bool arrived;
bool problem = false;


std::clock_t last_msg;
double duration;


void destination_callback(const geometry_msgs::Twist::ConstPtr &msg)
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
  x_point = dest_twist.linear.x;
  y_point = dest_twist.linear.y;
  last_msg = std::clock();

}

void position_callBack(const geometry_msgs::Twist::ConstPtr &msg)
{
  pos_twist = *msg;
  robot_x = pos_twist.linear.x;
  robot_y = pos_twist.linear.y;
  robot_theta = pos_twist.angular.z;
  last_msg = std::clock();
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_follower");

  ros::NodeHandle n("~");

  n.param<float>("/path_follower/angle_P", P, 0);
  n.param<float>("/path_follower/linear_speed", aver_lin_vel, 0);

  ros::Subscriber twist_sub = n.subscribe("/destination_point", 1, destination_callback);
  ros::Subscriber robot_position = n.subscribe("/deadreckogning/Pos", 1, position_callBack);
  ros::Subscriber breaker = n.subscribe("/break_info", 1, break_callBack);
  ros::Publisher desired_velocity = n.advertise<geometry_msgs::Twist>("/desired_velocity", 1);
  ros::Publisher dest_point = n.advertise<geometry_msgs::Twist>("/point_destination_robot", 1);
  ros::Publisher robot_arrived = n.advertise<std_msgs::Bool>("/robot_arrived", 1);
  ros::Publisher help = n.advertise<geometry_msgs::Twist>("/help_info", 1);

  ros::Rate loop_rate(freq);

  ROS_INFO("Running twist_interpreter");

  std_msgs::Bool is_robot_arrived;
  geometry_msgs::Twist help_msg;
  geometry_msgs::Twist point_plot;

  while (ros::ok())
  {
    ros::spinOnce();

    //Transforming the point in the robot frame
    x_point_robot = (x_point - robot_x) * cos(robot_theta) + (y_point - robot_y) * sin(robot_theta);
    y_point_robot = - (x_point - robot_x) * sin(robot_theta) + (y_point - robot_y) * cos(robot_theta);
    point_plot.linear.x = x_point_robot;
    point_plot.linear.y = y_point_robot;

    dist_left = sqrt(pow(x_point_robot,2) + pow(y_point_robot,2));
    diff_angle = findangle(x_point_robot, y_point_robot);

    if(dist_left < dist_threshold)
    {
        arrived = true;
        desire_vel.linear.x = 0;
        desire_vel.angular.z = 0;
    }

    if((!arrived)&&(!problem))
    {
      if(sgn(diff_angle)*diff_angle > angle_ref_max)
      {
        help_msg.linear.x = 1;
        desire_vel.linear.x = 0;
        desire_vel.angular.z = P * diff_angle;
      }
      else
      {
        help_msg.linear.x = 0;
        desire_vel.linear.x = aver_lin_vel;
        desire_vel.angular.z = P * diff_angle;
      }
    }
    else if(problem)
    {
      desire_vel.linear.x = 0;
      desire_vel.angular.z = 0;
    }

    // if(dist_left > dist_threshold)
    // {
    //   arrived = false;
    // }

    if(arrived){help_msg.angular.x = 1;}else{help_msg.angular.x = 0;}
    // if(problem){help_msg.linear.z = 1;}else{help_msg.linear.z = 0;}


    help_msg.angular.y = diff_angle;
    help_msg.angular.z = angle_ref_max;
    // help_msg.angular.x = x_point_robot;
    help_msg.linear.z = y_point_robot;

    is_robot_arrived.data = arrived;

    robot_arrived.publish( is_robot_arrived );
    dest_point.publish( point_plot);
    desired_velocity.publish(desire_vel);
    help.publish(help_msg);
    loop_rate.sleep();
  }

  return 0;
}
