#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float32MultiArray.h"
#include <cmath>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include "robo7_msgs/classifiedObj.h"
#include "robo7_msgs/aObject.h"
#include "robo7_msgs/allObjects.h"

std::vector<robo7_msgs::aObject> recieved_objects;

int number_objects = 0;
float object_height = 0; // 0.1;
float marker_height = 0.05; // 0.15;

class Objects
{
public:
  ros::Subscriber objects_sub;
  ros::Publisher marker_array_pub; //, marker_pub;
  int weight_lim;

  //Initialisation
  bool objects_received;

  Objects(ros::NodeHandle nh, ros::Publisher marker_array_pub)
  {
    // The application does not show objects with weights lower than this
    nh.param<int>("/all_objects/weight_lim", weight_lim, 4);

    this->objects_sub = nh.subscribe("/vision/all_objects", 1, &Objects::objectsCallback, this);

    this->marker_array_pub = marker_array_pub;

    this->objects_received = false;
  }


  void objectsCallback(const robo7_msgs::allObjects::ConstPtr &objects_msg)
  {
    recieved_objects.clear();

    for (int i = 0; i < objects_msg->objects.size(); i++)
    {
      recieved_objects.push_back(objects_msg->objects[i]);
    }

    objects_received = true;
  }


  void updateObjects(){
    if (objects_received){
      visualization_msgs::MarkerArray obj_points_msg;

      obj_points_msg.markers.resize(recieved_objects.size());

      for (int i = 0; i < recieved_objects.size(); i++)
      {
        // If an object does not have enogh total votes, do not visualize it.
        if (recieved_objects[i].total_votes < weight_lim){
          continue;
        }

        obj_points_msg.markers[i].header.frame_id = "/map";
        obj_points_msg.markers[i].header.stamp = ros::Time::now();
        obj_points_msg.markers[i].action = visualization_msgs::Marker::ADD;
        obj_points_msg.markers[i].pose.orientation.w = 1.0;
        obj_points_msg.markers[i].ns = "Found objects";
        obj_points_msg.markers[i].id = i;
        obj_points_msg.markers[i].type = visualization_msgs::Marker::SPHERE;

        obj_points_msg.markers[i].scale.x = 0.05;
        obj_points_msg.markers[i].scale.y = 0.05;
        obj_points_msg.markers[i].scale.z = 0.05;

        obj_points_msg.markers[i].color.r = 1.0;
        obj_points_msg.markers[i].color.g = .9;
        obj_points_msg.markers[i].color.b = .3;
        obj_points_msg.markers[i].color.a = 1.0;

        obj_points_msg.markers[i].pose.position.x = recieved_objects[i].pos.x;
        obj_points_msg.markers[i].pose.position.y = recieved_objects[i].pos.y;
        obj_points_msg.markers[i].pose.position.z = object_height;

        }

      marker_array_pub.publish(obj_points_msg);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "all_objects");
  ros::NodeHandle nh;

  ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("FoundObjects", 10);

  ROS_INFO("Init objects visualization");
  Objects objects = Objects(nh, marker_array_pub);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {

    objects.updateObjects();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
