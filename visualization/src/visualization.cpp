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
double control_frequency = 100.0;


class markerRviz
{
public:
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber robot_position1;
  ros::Subscriber robot_position3;
  ros::Publisher marker_parameters1;
  ros::Publisher marker_parameters2;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::TransformBroadcaster br2;
  tf::Transform transform2;
  tf::TransformBroadcaster br3;
  tf::Transform transform3;

  markerRviz()
  {
    n = ros::NodeHandle("~");

    x_angle = 0;
    y_angle = 0;
    z_pos = 0;
    nh.param<float>("/visualization/lidar_angle", lidar_angle, 0);

    robot_position1 = n.subscribe("/deadreckogning/Pos", 1, &markerRviz::deadReckogning_callBack, this);
    robot_position3 = n.subscribe("/deadreckogning/Pos2", 1, &markerRviz::deadReckogning3_callBack, this);

    marker_parameters1 = n.advertise<visualization_msgs::Marker>("robotMarker", 1);
    marker_parameters2 = n.advertise<visualization_msgs::Marker>("robotMarker2", 1);
  }

  void deadReckogning_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x_pos = msg->linear.x;
      y_pos = msg->linear.y;
      z_angle = msg->angular.z;
  }

  void deadReckogning3_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x3_pos = msg->linear.x;
      y3_pos = msg->linear.y;
      z3_angle = msg->angular.z;
  }

  void updatePosition(){
    //Generate the future published twist msg
    visualization_msgs::Marker marker;

    t = ros::Time::now();

    marker.header.frame_id = "laser";
    marker.header.stamp = t;
    marker.ns = "map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;
    marker.scale.x = 0.14;
    marker.scale.y = 0.14;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //Set the frame centered on the robot
    transform.setOrigin( tf::Vector3(x_pos, y_pos, z_pos) );
    tf::Quaternion q;
    q.setRPY(x_angle, y_angle, z_angle);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, t, "map", "laser"));

    // transform2.setOrigin( tf::Vector3(0, 0, 0) );
    // tf::Quaternion q2;
    // q2.setRPY(0, 0, lidar_angle);
    // transform2.setRotation(q2);
    // br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "robot", "laser"));

    transform3.setOrigin( tf::Vector3(x3_pos, y3_pos, z_pos) );
    tf::Quaternion q3;
    q3.setRPY(x_angle, y_angle, z3_angle);
    transform3.setRotation(q3);
    br3.sendTransform(tf::StampedTransform(transform3, t, "map", "robot2"));

    marker_parameters1.publish( marker );

    //Non-linear model robot marker
    marker.header.frame_id = "robot2";
    marker.color.g = 0;
    marker_parameters2.publish( marker );

  }


private:
  //Robot position parameters
  float x_pos;
  float y_pos;
  float z_pos;
  float x_angle;
  float y_angle;
  float z_angle;
  float lidar_angle;
  float x3_pos;
  float y3_pos;
  float z3_angle;

  //Time constant
  ros::Time() t;
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
