//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>


class PathPlanning;

class Node
{
    public:
        float x, y, theta;
        float control, time;
        float path_x, path_y, path_cost, cost_to_come;
        float tolerance_radius, tolerance_angle;

        Node(float x, float y, float theta, float control, float time, float path_x, float path_y, float path_cost, float cost_to_come)
        {
            this->x = x;
            this->y = y;
            this->theta = theta;
            this->control = control;
            this->time = time;
            this->path_x = path_x;
            this->path_y = path_y;
            this->path_cost = path_cost;
            this->cost_to_come = cost_to_come;

            this->parent = Null
            this->successor = Null

            this->tolerance_radius = 1e-1;
            this->tolerance_angle = pi/8;
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

    robot_position = nh.subscribe("/deadreckogning/Pos", 1000, &PathPlanning::getPositionCallBack, this);

    
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
