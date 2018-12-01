#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "robo7_msgs/XY_coordinates.h"
#include "robo7_msgs/classifiedObj.h"
#include "robo7_msgs/aObject.h"
#include "robo7_msgs/allObjects.h"
#include "robo7_msgs/allObstacles.h"
#include "robo7_srvs/SaveAll.h"


class ObjectSaver
{
  public:
	ros::NodeHandle n;
	ros::Subscriber objs_sub;
	ros::Subscriber obss_sub;
  ros::ServiceServer save_all_service;

	ObjectSaver()
	{
		// Parameters
    n.param<std::string>("/object_saver/objs_file", objs_file, "objs.txt");
    n.param<std::string>("/object_saver/obss_file", obss_file, "obss.txt");

		objs_sub = n.subscribe("/vision/all_objects", 1, &ObjectSaver::ObjsCallback, this);
		obss_sub = n.subscribe("/localization/mapping/the_obstacles", 1, &ObjectSaver::ObssCallback, this);
    save_all_service = n.advertiseService("/vision/save", &ObjectSaver::saveAllRequest, this);

    objs_recieved = false;
    obss_recieved = false;

	}


	void ObjsCallback(const robo7_msgs::allObjects::ConstPtr &allobjs){
    int num_objs = allobjs->objects.size();

    if (num_objs > 0){
      //ROS_INFO("New objects recieved");

      objs_recieved = true;
      recieved_objs.clear();

      for (int i = 0; i < num_objs; ++i){
        recieved_objs.push_back(allobjs->objects[i]);
      }
      //ROS_INFO("recieved_objs.size() %d", (int)recieved_objs.size());
    }
	}


  void ObssCallback(const robo7_msgs::allObstacles::ConstPtr &allobss){
    int num_obss = allobss->the_obstacles.size();

    if (num_obss > 0){
      //ROS_INFO("New obstacles recieved");

      obss_recieved = true;
      recieved_obss.clear();

      for (int i = 0; i < num_obss; ++i){
        recieved_obss.push_back(allobss->the_obstacles[i]);
      }
      obstacle_size = allobss->obstacle_size;
      //ROS_INFO("recieved_obss.size() %d", (int)recieved_obss.size());
    }
  }


  bool saveObjs(){
    std::ofstream objs_fs;
    objs_fs.open(objs_file.c_str());

    if (!objs_fs.is_open()){
        ROS_ERROR_STREAM("Could not read from "<<objs_file<<". Please double check that the file exists. Aborting.");
        return false;
    }

    for (int i = 0; i < recieved_objs.size(); ++i){
      robo7_msgs::aObject obj = recieved_objs[i];
      objs_fs << (int)obj.obj_class << " " << obj.pos.x << " " << obj.pos.y << " " << obj.total_votes << "\n";
    }

    objs_fs.close();
    return true;

  }

  bool saveObss(){
    std::ofstream obss_fs;
    obss_fs.open(obss_file.c_str());

    if (!obss_fs.is_open()){
        ROS_ERROR_STREAM("Could not read from "<<obss_file<<". Please double check that the file exists. Aborting.");
        return false;
    }

    for (int i = 0; i < recieved_obss.size(); ++i){
      geometry_msgs::Vector3 obs = recieved_obss[i];
      obss_fs << obstacle_size << " " << obs.x << " " << obs.y << "\n";
    }

    obss_fs.close();
    return true;

  }


  bool saveAllRequest(robo7_srvs::SaveAll::Request &req,
                robo7_srvs::SaveAll::Response &res){
    res.success = true;

    if (req.save_obj){
      if(!objs_recieved){
        ROS_WARN("Trying to save objects but no objects have been recieved");
        res.success = false;
        return true;
      }

      ROS_INFO("Saving objects");
      res.success = saveObjs();

    }

    if (req.save_obs){
      if(!obss_recieved){
        ROS_WARN("Trying to save obstacles but no obstacles have been recieved");
        res.success = false;
        return true;
      }

      ROS_INFO("Saving obstacles");
       res.success = saveObss();
    }

    return true;
  }



  private:
    std::string objs_file;
    std::string obss_file;
    bool objs_recieved;
    bool obss_recieved;
    std::vector<robo7_msgs::aObject> recieved_objs;
    std::vector<geometry_msgs::Vector3> recieved_obss;
    float obstacle_size;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_saver");

	ObjectSaver object_saver;

	ros::Rate loop_rate(10);

	ROS_INFO("Object saver running");

	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
