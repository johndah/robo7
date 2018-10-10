//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

double control_frequency = 10.0;

class PathPlanning
{
public:
  ros::NodeHandle nh;
  ros::Subscriber robot_position;


  //Initialisation
  float x, y, theta;

  PathPlanning()
  {
    nh = ros::NodeHandle("~");

    robot_position = nh.subscribe("/deadreckogning/Pos", 1000, &PathPlanning::getPositionCallBack, this);

    
    pi = 3.14159265358979323846;

    //Other parameters

    
    dt = 1/control_frequency;

    //robot_position = nh.advertise<geometry_msgs::Twist>("Pos", 1000);

  }

  void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x = msg->linear.x;
      y = msg->linear.y;
      theta = msg->angular.z;
  }

  float getPath(){
      return y;
  }

 

  void updatePosition(){
    //Generate the future published twist msg
    geometry_msgs::Twist twist_msg;

  

    //robot_position.publish(twist_msg);
  }

private:
  
  float pi, dt;
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathplanning");

    PathPlanning path_planning;
    float randomInt;

    ros::Rate loop_rate(control_frequency);

    while(path_planning.nh.ok())
    {
        randomInt = path_planning.getPath();
        ROS_INFO("y: %f", randomInt);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
