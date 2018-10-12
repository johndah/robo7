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
double control_frequency = 10000.0;


class lidar_plot
{
public:
  ros::NodeHandle n;
  ros::NodeHandle nh;
  tf::TransformBroadcaster br2;
  tf::Transform transform2;

  lidar_plot()
  {
    n = ros::NodeHandle("~");
    nh.param<float>("/lidar_plot/lidar_angle", lidar_angle, 0);
  }

  void updatePosition(){
    //Generate the future published twist msg

    transform2.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion q2;
    q2.setRPY(0, 0, lidar_angle);
    transform2.setRotation(q2);
    br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "robot", "laser"));
  }


private:
  //Time constant
  ros::Time t;
  float lidar_angle;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_plot");

    lidar_plot lidar_frame_;

    ros::Rate loop_rate(control_frequency);

    while(lidar_frame_.n.ok())
    {
        lidar_frame_.updatePosition();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
