//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>


class PathPlanning;

class Node
{
    public:
        float pi;
        float x, y, theta;

        Node(float x, float y, float theta)
        {
            this->x = x;
            this->y = y;
            this->theta = theta;
        }
};

double control_frequency = 10.0;


class PathPlanning
{
public:
  ros::NodeHandle nh;
  ros::Subscriber robot_position;


  //Initialisation
  float x0, y0, theta0;

  PathPlanning()
  {
    nh = ros::NodeHandle("~");

    robot_position = nh.subscribe("/deadreckogning/Pos1", 1000, &PathPlanning::getPositionCallBack, this);

    
    pi = 3.14159265358979323846;

    //Other parameters

    
    dt = 1/control_frequency;

    //robot_position = nh.advertise<geometry_msgs::Twist>("Pos", 1000);

  }

  void getPositionCallBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x0 = msg->linear.x;
      y0 = msg->linear.y;
      theta0 = msg->angular.z;
  }

  float getPath(){

      Node node = Node(x0, y0, theta0);
      ROS_INFO("Init node with 2x: %f", 2*x0);
      return 2*node.x;
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

    PathPlanning path_planning = PathPlanning();
    float randomInt;

    ros::Rate loop_rate(control_frequency);

    while(path_planning.nh.ok())
    {
        randomInt = path_planning.getPath();
        
        ROS_INFO("x: %f", randomInt);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
