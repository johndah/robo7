//Input all the libraries needed
#include <math.h>
#include <vector>
#include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>

// Control @ 10 Hz
double control_frequency = 10.0;


class Posvis
{
public:
  ros::NodeHandle n;
  ros::Subscriber obj_pos_sub;
  ros::Publisher obj_pos_rviz_pub;

  Posvis()
  {
    n = ros::NodeHandle("~");

    //XY_coordinates = n.subscribe("/scan_to_coordinates/point_cloud_coordinates", 1000, &laserXY::coordinates_callBack, this);
    obj_pos_sub = n.subscribe("/object/pos", 1, &Posvis::point_callBack, this);
    obj_pos_rviz_pub = n.advertise<geometry_msgs::PointStamped>("/object/pos_rviz", 1);
  }

  void point_callBack(const geometry_msgs::Point::ConstPtr &msg)
  {
      x_point = msg->x;
      y_point = msg->y;
      z_point = msg->z;
  }

  void updatePoint()
  {
    geometry_msgs::Point point;
    geometry_msgs::PointStamped pointer;
    std_msgs::Header _header;

    _header.frame_id = "cam";

    point.x = x_point;
    point.y = y_point;
    point.z = z_point;

    pointer.point = point;
    pointer.header = _header;

    obj_pos_rviz_pub.publish( pointer );

  }

private:
  //Robot position parameters
  float x_point;
  float y_point;
  float z_point;

};





int main(int argc, char **argv)
{
    ros::init(argc, argv, "obj_pos_vis");

    Posvis obj_pos_vis;

    ros::Rate loop_rate(control_frequency);

    while(obj_pos_vis.n.ok())
    {
        obj_pos_vis.updatePoint();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
