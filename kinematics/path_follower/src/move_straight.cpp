// #include <algorithm>
#include <stdlib.h>     /* abs */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robo7_msgs/destination_point.h>
#include <robo7_msgs/the_robot_position.h>
#include <robo7_srvs/MoveStraight.h>

float control_frequency = 50.0;
float pi = 3.14;
float dt = 1/control_frequency;

class straight_move
{
public:
  ros::NodeHandle n;
  ros::Publisher desired_velocity;
  ros::ServiceServer straight_line_service;

  straight_move()
  {
    //Initialisation
    n.param<float>("/move_straight/linear_velocity", linear_vel, 0.15);

    desired_velocity = n.advertise<geometry_msgs::Twist>("/desired_velocity", 1);

    straight_line_service = n.advertiseService("/kinematics/path_follower/straight_move", &straight_move::straight_Sequence, this);
  }

  bool straight_Sequence(robo7_srvs::MoveStraight::Request &req,
         robo7_srvs::MoveStraight::Response &res)
  {
    //Pick back the desired angle
    float desire_distance = req.desired_distance;
    bool move_back = req.move_backward;
    float desire_way = 0;

    if(move_back)
    {
        desire_way = -1;
    }
    else
    {
      desire_way = 1;
    }

    float time_moving = desire_distance / linear_vel;

    ROS_INFO("Time moving : %lf", time_moving);

    ros::Rate loop_rate(10/time_moving);

    geometry_msgs::Twist desire_vel;

    //Publish the velocities
    desire_vel.linear.x = desire_way * linear_vel;
    desire_vel.angular.z = 0;

    for(int i=0; i<10; i++)
    {
      desired_velocity.publish( desire_vel );
      loop_rate.sleep();
    }


    //Then stop the translation movement
    desire_vel.linear.x = 0;
    desire_vel.angular.z = 0;
    desired_velocity.publish( desire_vel );

    res.success = true;
  }


private:
  //Point destination data
  float linear_vel;

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "straight_move");

    straight_move straight_move_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Straight Move running");

    ros::spin();

    return 0;
}
