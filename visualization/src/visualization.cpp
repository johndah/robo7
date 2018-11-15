//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <robo7_msgs/former_position.h>
#include <robo7_msgs/the_robot_position.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

// Control @ 10 Hz
double control_frequency = 30.0;


class markerRviz
{
public:
  ros::NodeHandle n;
  ros::NodeHandle nh;
  //Subscribers
  ros::Subscriber robot_position1;
  ros::Subscriber robot_position2;
  //Publishers
  ros::Publisher marker_parameters1;
  ros::Publisher marker_parameters2;
  ros::Publisher lidar_pub;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::TransformBroadcaster br2;
  tf::Transform transform2;

  markerRviz()
  {
    n = ros::NodeHandle("~");

    x_angle = 0;
    y_angle = 0;
    z_pos = 0;
    x2_angle = 0;
    y2_angle = 0;
    z2_pos = 0;
    nh.param<float>("/visualization/lidar_angle", lidar_angle, 0);

    robot_position1 = n.subscribe("/localization/kalman_filter/position_timed", 1, &markerRviz::deadReckoning_callBack, this);
    robot_position2 = n.subscribe("/localization/icp/position", 1, &markerRviz::deadReckoning2_callBack, this);

    marker_parameters1 = n.advertise<visualization_msgs::Marker>("robotMarker", 1);
    marker_parameters2 = n.advertise<visualization_msgs::Marker>("robotMarker2", 1);
    marker_parameters2 = n.advertise<sensor_msgs::LaserScan>("/visualization/lidar_scan", 1);
  }

  void deadReckoning_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
      robot_position = *msg;
      x_pos = robot_position.position.linear.x;
      y_pos = robot_position.position.linear.y;
      z_angle = robot_position.position.angular.z;
  }

  void deadReckoning2_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x2_pos = msg->linear.x;
      y2_pos = msg->linear.y;
      z2_angle = msg->angular.z;
  }

  void updatePosition()
  {
    //Generate the future published twist msg
    visualization_msgs::Marker marker;

    t = ros::Time::now();
    // ROS_INFO("time, pose : %lf, %lf, %lf, %lf", t.toSec(), x_pos, y_pos, z_angle);

    // marker.header.frame_id = "robot";
    // marker.header.stamp = t;
    // marker.ns = "map";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    // marker.mesh_resource = "package://visualization/model/robo7.dae";
    // marker.mesh_use_embedded_materials = true;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;
    // marker.pose.orientation.x = 0;
    // marker.pose.orientation.y = 0;
    // marker.pose.orientation.z = 1.57;
    // marker.pose.orientation.w = 1.57;
    // marker.scale.x = 0.01;
    // marker.scale.y = 0.01;
    // marker.scale.z = 0.01;
    // marker.color.a = 0.0;
    // marker.color.r = 0.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;

    marker.header = robot_position.header;
    marker.header.frame_id = "robot";
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
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    //Extract the corresponding laser scan
    lidar_scan = the_robot_position.the_lidar_scan;
    lidar_scan.header.stamp = t;

    //Set the frame centered on the robot
    transform.setOrigin( tf::Vector3(x_pos, y_pos, z_pos) );
    tf::Quaternion q;
    q.setRPY(x_angle, y_angle, z_angle);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, t, "map", "robot"));

    //Set the frame centered on the robot
    transform2.setOrigin( tf::Vector3(x2_pos, y2_pos, z2_pos) );
    tf::Quaternion q2;
    q2.setRPY(x2_angle, y2_angle, z2_angle);
    transform2.setRotation(q2);
    br2.sendTransform(tf::StampedTransform(transform2, t, "map", "robot_corrected"));

    marker_parameters1.publish( marker );
    lidar_pub.publish( lidar_scan );

    marker.header.frame_id = "robot_corrected";
    marker.color.g = 1.0;
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
  float x2_pos;
  float y2_pos;
  float z2_pos;
  float x2_angle;
  float y2_angle;
  float z2_angle;

  robo7_msgs::the_robot_position robot_position;
  sensor_msgs::LaserScan lidar_scan;

  //Time constant
  ros::Time t;
};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualization");

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
