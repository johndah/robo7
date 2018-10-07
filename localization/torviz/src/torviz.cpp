//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// Control @ 10 Hz
double control_frequency = 10.0;


class markerRviz
{
public:
  ros::NodeHandle n;
  ros::Subscriber robot_position;
  ros::Publisher marker_parameters;
  tf::TransformBroadcaster br;
  tf::Transform transform;

  markerRviz()
  {
    n = ros::NodeHandle("~");

    x_angle = 0;
    y_angle = 0;
    z_pos = 0;

    robot_position = n.subscribe("/deadreckogning/Pos", 1000, &markerRviz::deadReckogning_callBack, this);

    marker_parameters = n.advertise<visualization_msgs::Marker>("robotMarker", 1000);
  }

  void deadReckogning_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x_pos = msg->linear.x;
      y_pos = msg->linear.y;
      z_angle = msg->angular.z;
  }

  void updatePosition(){
    //Generate the future published twist msg
    visualization_msgs::Marker marker;

    //Transform angles to quaternion
    // quater = new quaternion;
    float s = sin(z_angle/2);


    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = y_pos;
    marker.pose.position.z = z_pos;
    marker.pose.orientation.x = s * x_pos;
    marker.pose.orientation.y = s * y_pos;
    marker.pose.orientation.z = s * z_pos;
    marker.pose.orientation.w = cos(z_angle / 2);
    marker.scale.x = 0.14;
    marker.scale.y = 0.14;
    marker.scale.z = 0.18;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //Set the frame centered on the robot
    transform.setOrigin( tf::Vector3(x_pos, y_pos, z_pos) );
    transform.setRotation( tf::Quaternion(x_angle, y_angle, z_angle*s, cos(z_angle/2)) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "laser"));

    marker_parameters.publish( marker );

  }


private:
  //Robot position parameters
  float x_pos;
  float y_pos;
  float z_pos;
  float x_angle;
  float y_angle;
  float z_angle;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "torviz");

    markerRviz marker_rviz;

    ros::Rate loop_rate(control_frequency);

    while(marker_rviz.n.ok())
    {
        marker_rviz.updatePosition();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
