//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

// Control @ 10 Hz
double control_frequency = 10.0;


class robotRviz
{
public:
  ros::NodeHandle n;
  ros::Subscriber robot_position;
  ros::Publisher robot_to_rviz;

  robotRviz()
  {
    n = ros::NodeHandle("~");

    x_angle = 0;
    y_angle = 0;
    z_pos = 0;

    robot_position = n.subscribe("deadReckogning/pos", 1000, &robotRviz::deadReckogning_callBack, this);

    robot_to_rviz = n.advertise<nav_msgs::Odometry>( "/odom", 0 );
  }

  void deadReckogning_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x_pos = msg->linear.x;
      y_pos = msg->linear.y;
      z_angle = msg->angular.z;
  }

  void updatePosition(){
    //Generate the future published twist msg
    nav_msgs::Odometry odom_msg;
    std_msgs::Header header_;
    geometry_msgs::PoseWithCovariance pose_;

    //Defining header
    header_.stamp = ros::Time(0);
    header_.frame_id = "map";

    //Defining Point position
    pose_.pose.position.x = x_pos;
    pose_.pose.position.y = y_pos;
    pose_.pose.position.z = z_pos;

    //Orientation
    s = sin(x_angle/2);
    pose_.pose.orientation.x = x_angle * s;
    pose_.pose.orientation.y = y_angle * s;
    pose_.pose.orientation.z = z_angle * s;
    pose_.pose.orientation.w = cos(x_angle/2);

    //set the pose_ in msg
    odom_msg.pose = pose_;


    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    robot_to_rviz.publish( odom_msg );

  }


private:
  //Robot position parameters
  float x_pos;
  float y_pos;
  float z_pos;
  float x_angle;
  float y_angle;
  float z_angle;
  float s;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotrviz");

    robotRviz robot_rviz;

    ros::Rate loop_rate(control_frequency);

    while(robot_rviz.n.ok())
    {
        robot_rviz.updatePosition();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
