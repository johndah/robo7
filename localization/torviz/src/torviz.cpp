//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

// Control @ 10 Hz
double control_frequency = 10.0;

// class quaternion
// {
// public:
//   float x;
//   float y;
//   float z;
//   float w;
//
//   quaternion()
//   {
//     x = 0;
//     y = 0;
//     z = 0;
//     w = 0;
//   }
//
//   void updateValues(float x1, float y1, float z1, float w1)
//   {
//     x = x1;
//     y = y1;
//     z = z1;
//     w = w1;
//   }
// };

class markerRviz
{
public:
  ros::NodeHandle n;
  ros::Subscriber robot_position;
  ros::Publisher marker_parameters;

  markerRviz()
  {
    n = ros::NodeHandle("~");

    x_angle = 0;
    y_angle = 0;
    z_pos = 0;

    robot_position = n.subscribe("deadReckogning/pos", 1000, &markerRviz::deadReckogning_callBack, this);

    marker_parameters = n.advertise<visualization_msgs::Marker>( "robotMarker", 0 );
  }

  void deadReckogning_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      x_pos = msg->linear.x;
      y_pos = msg->linear.y;
      angle_pos = msg->angular.z;
  }

  void updatePosition(){
    //Generate the future published twist msg
    visualization_msgs::Marker marker;

    //Transform angles to quaternion
    // quater = new quaternion;
    float s = sin(angle_pos/2);


    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = y_pos;
    marker.pose.position.z = z_pos;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = angle_pos * s;
    marker.pose.orientation.w = cos(angle_pos / 2);
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    marker_parameters.publish( marker );

  }

  // assumes axis is already normalised
  // void changeToQuaternion(float ax, float ay, float az) {
  //
  //   // x = ax * s;
  //   // y = ay * s;
  //   // z = az * s;
  //   // w = cos(a1.angle/2);
  //   quater.updateValues(ax * s, ay * s, az * s, cos(ax/2));
  // }

private:
  //Robot position parameters
  float x_pos;
  float y_pos;
  float z_pos;
  float x_angle;
  float y_angle;
  float angle_pos;

  // quaternion quater = new quaternion();
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
