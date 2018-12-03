#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "robo7_msgs/XY_coordinates.h"
#include "robo7_msgs/classifiedObj.h"
#include "robo7_msgs/aObject.h"
#include "robo7_msgs/allObjects.h"
#include "robo7_msgs/the_robot_position.h"
#include "robo7_srvs/objectToRobot.h"
#include "robo7_srvs/FilterOn.h"


class ObjectFilter
{
  public:
	ros::NodeHandle n;
  ros::ServiceClient obj_to_robo_srv;
  ros::ServiceServer filtered_objs_srv_server;
	ros::Subscriber obj_sub;
  ros::Subscriber robo_pos_sub;
  ros::Publisher all_obj_pub;
  ros::Publisher speaker_pub;


	ObjectFilter()
	{
		// Parameters
    n.param<int>("/object_filter/num_classes", num_classes, 14);

    // If distance between two objects of the same class is smaller than this, consider them the same obj
		n.param<float>("/object_filter/dist_same_class_lim", dist_same_class_lim, 0.08);

    // If distance between two objects of the different classes is smaller than this, consider them the same obj
		n.param<float>("/object_filter/dist_other_obj_lim", dist_other_class_lim, 0.04);

    filtered_objs_srv_server = n.advertiseService("/object_filter/activate", &ObjectFilter::turnOnFilter, this);



		obj_sub = n.subscribe("/vision/results", 1, &ObjectFilter::ObjCallback, this);
    robo_pos_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &ObjectFilter::roboPosCallback, this);
    obj_to_robo_srv = n.serviceClient<robo7_srvs::objectToRobot>("/localization/object_to_robot");
		all_obj_pub = n.advertise<robo7_msgs::allObjects>("/vision/all_objects", 1);
    speaker_pub = n.advertise<std_msgs::Int16>("/vision/object/class", 1);

    robot_position_set = false;
    filter_on = false;

	}

  bool turnOnFilter(robo7_srvs::FilterOn::Request &req,
         robo7_srvs::FilterOn::Response &res)
	{

		float time_req = req.time;

    ros::Rate loop_rate(50/time_req);

    filter_on = true;

    for( int i=0; i < 50; i++){
      ros::spinOnce();
      loop_rate.sleep();
    }

    filter_on = false;

		res.success = true;

    publishObjects();

    return true;
	}


  void roboPosCallback(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    robo_pos = msg->position;
    robot_position_set = true;
  }


	void ObjCallback(const robo7_msgs::classifiedObj::ConstPtr &msg)
  {
    if(filter_on)
    {
      if (!robot_position_set){
        ROS_WARN("Object filter: Unable to filer object, no robot position recieved");
        publishSpeaker(-1);
        return;
      }

      if (msg->pos.x == 0 && msg->pos.y == 0 && msg->pos.z == 0){
        ROS_WARN("Object filter: Unable to filer object, 0,0,0 in camera frame recieved");
        publishSpeaker(-1);
        return;
      }

      robo7_srvs::objectToRobot::Request srv_req;
      robo7_srvs::objectToRobot::Response srv_resp;

      srv_req.camera_position = msg->pos;         // geometry_msgs/Point
      srv_req.robot_position = robo_pos;          // geometry_msgs/Twist

      obj_to_robo_srv.call(srv_req, srv_resp);

      if (!srv_resp.success){
        ROS_WARN("Object filter: Unable to filer object, objectToRobot service call failed");
        publishSpeaker(-1);
        return;
      }
      // ROS_INFO("Object filter: new object position in robot x: %f", srv_resp.object_in_robot_frame.x);
      // ROS_INFO("Object filter: new object position in robot y: %f", srv_resp.object_in_robot_frame.y);
      // ROS_INFO("Object filter: new object position in robot z: %f", srv_resp.object_in_robot_frame.z);
      // ROS_INFO("Object filter: new object position in map x: %f", srv_resp.object_in_map_frame.x);
      // ROS_INFO("Object filter: new object position in map y: %f", srv_resp.object_in_map_frame.y);

      std::vector<int> init_weigh(num_classes, 0);

      robo7_msgs::aObject new_obj;
      new_obj.obj_class = msg->objClass;
      new_obj.pos = srv_resp.object_in_map_frame;
      new_obj.total_votes = 1;
      new_obj.weights = init_weigh;
      new_obj.weights[msg->objClass] = 1;

      saveObj(new_obj);
    }
  }


  void publishObjects(){
    if(filtered_objs.size() > 0){
      robo7_msgs::allObjects pub_obj;
      pub_obj.objects = filtered_objs;
      all_obj_pub.publish(pub_obj);
    }
  }

  void publishSpeaker(int speak_class){
    std_msgs::Int16 pub_speak;
    pub_speak.data = speak_class;
    speaker_pub.publish(pub_speak);
  }


  float distance(robo7_msgs::aObject point_a, robo7_msgs::aObject point_b){
    // returns the distance between two aobjects
    return sqrt(pow(point_a.pos.x - point_b.pos.x, 2) + pow(point_a.pos.y - point_b.pos.y, 2));
  }


  int donminantClass(robo7_msgs::aObject a_obj){
    int size = a_obj.weights.size();
    int dom_class = 0;
    int dom_votes = 0;

    if (size > 0){
      for (int m = 0 ; m < size ; m++){
        if (a_obj.weights[m] > dom_votes){
          dom_votes = a_obj.weights[m];
          dom_class = m;
        }
      }

      // Speak if dominant class has changed:
      if(a_obj.obj_class != dom_class){
        publishSpeaker(dom_class);
      }

      return dom_class;

    } else {
      publishSpeaker(-1);
      ROS_WARN("Object filter: Something went wrong, comparing classes");
      return -1;
    }
  }


  void saveObj(robo7_msgs::aObject new_obj){
    // Do we have any objects in the vector already?
    float distance_lim = 0;
    if (filtered_objs.size() > 0){

      // Check all the current save object to see if the new_obj is the same
      for(std::vector<robo7_msgs::aObject>::iterator a_obj = filtered_objs.begin(); a_obj != filtered_objs.end(); ++a_obj) {

        // if its within the distance limmit it is considered the same object!
        float dist = distance(new_obj, *a_obj);
        //ROS_INFO("Object filter: new_obj distance: %f", dist);

        // Different limits to treat the new object as a separate object if it is the same class
        if (new_obj.obj_class == (*a_obj).obj_class){
          distance_lim = dist_same_class_lim;
          //ROS_INFO("Object filter: new object is the same class as another in the list");
        } else{
          distance_lim = dist_other_class_lim;
          //ROS_INFO("Object filter: new object is of another class a object in the list");
        }

        if (dist <= distance_lim){
          //ROS_INFO("Object filter: new object within the radius of another one");

          // avrage the positon
          (*a_obj).pos.x = (((*a_obj).pos.x * (*a_obj).total_votes) + new_obj.pos.x) / ((*a_obj).total_votes + 1);
          (*a_obj).pos.y = (((*a_obj).pos.y * (*a_obj).total_votes) + new_obj.pos.y) / ((*a_obj).total_votes + 1);

          (*a_obj).total_votes += 1;
          (*a_obj).weights[new_obj.obj_class] += 1;

          // Set class to dominant class
          int dom_class = donminantClass(*a_obj);
          if (dom_class != -1){
            (*a_obj).obj_class = dom_class;
          } else {
            ROS_WARN("Object filter: Error saving object");
          }
          return;
        }
      }
    }

    // We will only get here if the new_obj has not been matched to another object in the list
    ROS_INFO("Object filter: new object not matching any in list or list was empty, adding it");
    filtered_objs.push_back(new_obj);
    publishSpeaker(new_obj.obj_class);

  }


  private:
    int num_classes;
    float dist_same_class_lim;
    float dist_other_class_lim;
    geometry_msgs::Twist robo_pos;
    bool position_updated;
    bool robot_position_set;
    bool filter_on;
    std::vector<robo7_msgs::aObject> filtered_objs;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_filter");

	ObjectFilter object_filter;

	// ros::Rate loop_rate(10);

	ROS_INFO("Object filter running");

	// while (ros::ok()){
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
  //
	// }

	ros::spin();

	return 0;
}
