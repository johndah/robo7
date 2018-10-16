#include <algorithm>
#include <stdlib.h>     /* abs */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "robo7_msgs/WheelAngularVelocities.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/LaserScan.h>

geometry_msgs::Twist dest_twist;
geometry_msgs::Twist pos_twist;
geometry_msgs::Twist desire_vel;

int freq = 30;
float pi = 3.14159;
int break_scalar = 5;

double msg_timeout = 0.004; // acepted delay between msgs before stopping

// From rosparam get /robot_description/
double wheel_separation = 0.2198; // center of track to center of track
double wheel_radius = 0.049;
double max_allowed_speed = 25; //rad.s-1
int way_moving;
float L_vel;
float R_vel;
float lin_velocity;
float lin_threshold = 0.1;

//Ellipse form of non possible zone
float a = 0.15 + 0.2;
float b = 0.10;
float c = sqrt(pow(a,2)-pow(b,2));

//Couting points
int M_close;      //lidar
int M_max = 10;
int M_cam_close;      //camera
int M_cam_max = 10;

//Breaker boolean
bool break_info;
bool obstacle_cam;

//Datas extracted from LaserScan
float angle_min;
float angle_max;
float range_min;
float range_max;
float angle_increment;
std::vector<float> points_distances;
std::vector<float> points_intensities;
std::vector<float> angle;
float lidar_angle = pi;
float angle_;


std::clock_t last_msg;
double duration;


void laser_scan_callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    points_distances = msg->ranges;
    points_intensities = msg->intensities;
    range_min = msg->range_min;
    range_max = msg->range_max;
}

void obstacle_callBack(const std_msgs::Bool::ConstPtr &msg)
{
  obstacle_cam = msg->data;
}

void vel_L_callBack(const std_msgs::Float32::ConstPtr &msg)
{
  L_vel = msg->data;
}

void vel_R_callBack(const std_msgs::Float32::ConstPtr &msg)
{
  R_vel = msg->data;
}

float linear_speed(float left_wheel_speed, float right_wheel_speed)
{
  return wheel_radius*(right_wheel_speed + left_wheel_speed)/2;
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "breaker");

  ros::NodeHandle n;

  ros::Subscriber laser_scan = n.subscribe("/scan", 1, &laser_scan_callBack);
  ros::Subscriber estim_L_om = n.subscribe("/l_motor/estimated_vel", 1, &vel_L_callBack);
  ros::Subscriber estim_R_om = n.subscribe("/r_motor/estimated_vel", 1, &vel_R_callBack);
  ros::Subscriber obstacle_sub = n.subscribe("/obstacle/flag", 1, &obstacle_callBack);
  ros::Publisher breaker = n.advertise<std_msgs::Bool>("/break_info", 1);
  // ros::Publisher help = n.advertise<geometry_msgs::Twist>("/help_info", 1);

  ros::Rate loop_rate(freq);

  std_msgs::Bool bool_msg;
  geometry_msgs::Twist help_msg;


  while (ros::ok())
  {
    ros::spinOnce();

    lin_velocity = linear_speed(L_vel, -R_vel);
    if(lin_velocity < lin_threshold)
    {
      way_moving = 0;
    }
    else
    {
      way_moving = sgn(lin_velocity);
    }



    //Number of points too close
    M_close = 0;

    //Counting the number of point in the danger zone of the robot
    if(way_moving < 0)
    {
      for(int i=0; i<points_distances.size(); i++)
      {
        angle_ = angle_min + i*angle_increment;
        if((points_intensities[i]!=0)
            &&((wrapAngle(angle_) < wrapAngle(-pi/2 + lidar_angle))||(wrapAngle(angle_) > wrapAngle(pi/2 + lidar_angle)))
              &&(points_distances[i] < sqrt ( pow(a*cos(angle_),2) + pow(b*sin(angle_),2))))
              {
                M_close++;
              }
      }
      if(obstacle_cam)
      {
        M_close += M_max;
      }
    }
    else if(way_moving > 0)
    {
      for(int i=0; i<points_distances.size(); i++)
      {
        angle_ = angle_min + i*angle_increment;
        if((points_intensities[i]!=0)
            &&((wrapAngle(angle_) > wrapAngle(-pi/2 + lidar_angle))&&(wrapAngle(angle_) < wrapAngle(pi/2 + lidar_angle)))
              &&(points_distances[i] < sqrt ( pow(a*cos(angle_),2) + pow(b*sin(angle_),2))))
              {
                M_close++;
              }
      }
      if(obstacle_cam)
      {
        M_close += M_max;
      }
    }



    if(M_close > M_max){break_info = true;}
    if(M_close < M_max){break_info = false;}

    bool_msg.data = break_info;

    help_msg.linear.x = way_moving;
    help_msg.linear.y = lin_velocity;
    help_msg.linear.z = lin_threshold;
    help_msg.angular.x = L_vel;
    help_msg.angular.y = R_vel;
    help_msg.angular.z = lin_threshold;

    breaker.publish(bool_msg);
    // help.publish(help_msg);
    loop_rate.sleep();

  }

  return 0;
}
