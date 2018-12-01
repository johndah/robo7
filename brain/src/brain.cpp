#include "ros/ros.h"
#include "iostream"
#include "fstream"
#include "std_msgs/Bool.h"
#include "robo7_msgs/aObject.h"
#include "robo7_msgs/allObjects.h"
#include "robo7_srvs/distanceTo.h"
#include "robo7_srvs/IsGridOccupied.h"
#include "robo7_srvs/GoTo.h"
#include "robo7_srvs/PickupAt.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

float pi = 3.14159265359;

class Brain
{
public:
	ros::NodeHandle n;
	ros::Subscriber objects_sub;
	ros::Subscriber e_break_sub;
	ros::Subscriber robo_pos_sub;
	ros::ServiceClient distance_srv;
	ros::ServiceClient occupancy_srv;
	ros::ServiceClient go_to_srv;
	ros::ServiceClient pickup_at_srv;
	static int objs_values[];


	Brain()
	{
		n.param<int>("/brain/weight_thresh", weight_thresh, 5);
		n.param<float>("/brain/robot_pose_dist", robot_pose_dist, 0.25); // distance avay from the robot center to search for a pose
    n.param<std::string>("/brain/objs_file", objs_file, "objs.txt");

		distance_srv = n.serviceClient<robo7_srvs::distanceTo>("/distance_grid/distance");
		occupancy_srv = n.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
		go_to_srv = n.serviceClient<robo7_srvs::GoTo>("/kinematics/go_to");
		pickup_at_srv = n.serviceClient<robo7_srvs::PickupAt>("/gate_controller/pickup_at");
		robo_pos_sub = n.subscribe("/localization/kalman_filter/position", 1, &Brain::roboPosCallback, this);

		robot_position_set = false;
		go_to_pose.linear.x == -1;

		stop_seq = false;
		got_obj_dest = false;
		carrying_object = false;
		e_break = false;
		at_home = false;
		at_object = false;

		readObjsfile();

		ROS_INFO("Read %d objects from file.", (int)read_objs.size());
	}


	bool readObjsfile(){
		std::string line;

		std::ifstream objs_fs;
		objs_fs.open(objs_file.c_str());

		if (!objs_fs.is_open()){
				ROS_ERROR("Could not read objects file!");
				return false;

		} else{
			ROS_INFO("Reading objects from file...");

			while (getline(objs_fs, line)){
					if (line[0] == '#') {
						// comment -> skip
						continue;
					}

					float max_num = std::numeric_limits<float>::max();
					int max_num_int = std::numeric_limits<int>::max();

					float obj_x = max_num,
								 obj_y = max_num;

					int obj_cl = max_num_int,
							votes = max_num_int;

					std::istringstream line_stream(line);

					line_stream >> obj_cl >> obj_x >> obj_y >> votes;

					if ((obj_cl == max_num_int) || (obj_x == max_num) || (obj_y == max_num) || (votes == max_num_int)){
							ROS_WARN("Segment error in objs file. Skipping line: %s",line.c_str());
							continue;
					}

					robo7_msgs::aObject a_object;

					a_object.obj_class = obj_cl;
					a_object.pos.x = obj_x;
					a_object.pos.y = obj_y;
					a_object.total_votes = votes;

					if (votes < weight_thresh){
						continue;
					}

					read_objs.push_back(a_object);

			}
			return true;
		}
	}


	void roboPosCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    robo_pos = *msg;
    robot_position_set = true;
  }


	static bool comparePoints( const robo7_msgs::aObject & ob1, const robo7_msgs::aObject & ob2) {
		// Function to sort the different objects for execution
		int ob1_points = objs_values[ob1.obj_class];
		int ob2_points = objs_values[ob2.obj_class];

		//TODO: implement distance here somewhere
		// getDistance(ob1.pos.x, ob1.pos.y) < getDistance(ob2.pos.x, ob2.pos.y)

  	if( ob1_points != ob2_points){
			return (ob1_points > ob2_points);
		}
  	return (ob1.total_votes < ob2.total_votes);
	}


	float getDistance(float x, float y){
		robo7_srvs::distanceTo::Request srv_req;
		robo7_srvs::distanceTo::Response srv_resp;

		if (!robot_position_set){
			ROS_WARN("Brain: Trying to evaluate distance to objects without knowing the robot position");
			return -1;
		}

		srv_req.x_from = robo_pos.linear.x;
		srv_req.y_from = robo_pos.linear.y;
		srv_req.x_to = x;
		srv_req.y_to = y;

		distance_srv.call(srv_req, srv_resp);

		return srv_resp.distance;
	}


	float getOccupancy(float x, float y){
		robo7_srvs::IsGridOccupied::Request srv_req;
		robo7_srvs::IsGridOccupied::Response srv_resp;

		srv_req.x = x;
		srv_req.y = y;

		occupancy_srv.call(srv_req, srv_resp);

		return srv_resp.occupancy;
	}


	bool callGoTo( geometry_msgs::Twist to_pose ){
		robo7_srvs::GoTo::Request srv_req;
		robo7_srvs::GoTo::Response srv_resp;

		srv_req.robot_pose = robo_pos;
		srv_req.destination_pose = to_pose;

		occupancy_srv.call(srv_req, srv_resp);

		return srv_resp.success;
	}


	bool callPickupAt( geometry_msgs::Twist pickup_pose ) {
		robo7_srvs::PickupAt::Request srv_req;
		robo7_srvs::PickupAt::Response srv_resp;

		srv_req.object_pos = pickup_pose;

		pickup_at_srv.call(srv_req, srv_resp);

		return srv_resp.success;
	}


	int sgn(float v) {
	  if (v < 0) return -1;
	  else if (v > 0) return 1;
	  else return 0;
	}


	float findAngle(float x_obj, float y_obj, float x_rob, float y_rob) {
		float x = x_obj - x_rob;
		float y = y_obj - y_rob;

	  if(x==0){
	    return pi*sgn(y);

	  } else if((x<0)&&(y>0)){
	    return atan(y/x) + pi;

	  } else if ((x<0)&&(y<0)){
	    return atan(y/x) - pi;

	  } else {
	    return atan(y/x);
	  }
	  // return atan(x/y);
	}


	void setBestObject(){
		ROS_INFO("Evaluating objects");
		int num_objs = read_objs.size();

		if (!num_objs > 0){
			ROS_ERROR("Brain: No objects to evaluate!");
			return;
		}

		std::sort( read_objs.begin(), read_objs.end(), comparePoints );

		best_object = read_objs[0];

		// remove the object from the read ones
		read_objs.erase(read_objs.begin());

		//ROS_INFO("best object class: %d", best_object.obj_class);
	}


	geometry_msgs::Twist getDestPose(float x_dest, float y_dest) {
		geometry_msgs::Twist pose;
		pose.linear.x = -1;

		float lowest_occupancy = 1.0;

		// returns a pose that is possible to travel to, given the destination
		for (float alp = 0 ; alp < 2*pi ; alp = alp+(pi/4)){

			float test_x = robot_pose_dist * sin(alp);
			float test_y = robot_pose_dist * sin(alp);

			float new_occupancy = getOccupancy(test_x, test_y);

			if(new_occupancy < lowest_occupancy){
				ROS_INFO("Brain:getDestPose found a better pose");
				lowest_occupancy = new_occupancy;
				pose.linear.x = test_x;
				pose.linear.y = test_y;
			}
		}

		// SET POSE ROTATION!

		return pose;

	}



	void setStateRun(){
		ROS_INFO("Setting state");

		// if(e_break){
		// 	state = "ST_HANDLE_E_BREAK";
		// }

		if (stop_seq){
			// when all is done
			state = "ST_STOP_SEQ";
		}

		if(!got_obj_dest &! carrying_object){
		 	state = "ST_EVALUATE";
		}

		if(got_obj_dest &! carrying_object){
		 	state = "ST_GO_TO_OBJECT";
		}

		if(at_object){
			state = "ST_PICK_UP_OBJ";
		}

		if (carrying_object &! at_home){
			state = "ST_GO_HOME";
		}

		if (carrying_object && at_home){
			state = "ST_DROP_OBJ";
		}

	}


	void act(){
			if (state == "ST_EVALUATE"){
				ROS_INFO("ST_EVALUATE");
				// Start of the runing  mode

				while (go_to_pose.linear.x == -1 && read_objs.size() > 0){

					setBestObject();
					go_to_pose = getDestPose(best_object.pos.x, best_object.pos.y);

					if(go_to_pose.linear.x == -1){
						ROS_WARN("Brain: Failed to find robot pose for object, going to next object");
					}
				}

				if(go_to_pose.linear.x == -1){
					ROS_WARN("Brain: No more objects, stopping sequence");
					stop_seq = true;
				} else{
					got_destination = true;
				}

			}

			if (state == "ST_GO_TO_OBJECT"){
				ROS_INFO("ST_GO_TO_OBJECT");

				if(callGoTo(go_to_pose)){
					at_object = true;
				} else{
					// WHAT TO DO HERE?!
					ROS_WARN("Brain: we did not reach the object");
				}

			}

			if (state == "ST_PICK_UP_OBJ"){
				ROS_INFO("ST_PICK_UP_OBJ");


				// if (ros::service::call("/gate_controller/pickup_at", object_pos)){
				// 	carrying_object = true;
				// 	//best_object = NULL;
				// 	at_object = false;
				// }
				// else {
				// 	ROS_WARN("PICKUP FAILED!");
				// 	at_object = false;
				// }

			}


			if (state == "ST_DROP_OBJ"){
				ROS_INFO("ST_DROP_OBJ");

				// Drop up an object

				// go home after object is picked up
				carrying_object = false;
		  }


			if (state == "ST_GO_HOME"){
				ROS_INFO("ST_GO_HOME");

				// Go home to drop of object
				at_home = true;
			}


			// if (state == "ST_HANDLE_E_BREAK"){
			// 	ROS_INFO("ST_HANDLE_E_BREAK");
			//
			// 	// The emergency break was triggered!
			// 	ROS_INFO("Brain sees that the ebreak is triggered");
			//
			//
			// 	e_break = false;
			//
			// }

	}

private:
	std::string state;
	geometry_msgs::Twist robo_pos;
	geometry_msgs::Twist go_to_pose;
	robo7_msgs::aObject best_object;
	std::vector<robo7_msgs::aObject> read_objs;
	bool robot_position_set;
	bool got_destination;
	bool carrying_object;
	bool got_obj_dest;
	bool e_break;
	bool at_home;
	bool at_object;
	bool stop_seq;
	int weight_thresh;
	std::string objs_file;
	float robot_pose_dist;


};

int Brain::objs_values[14] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "brain");

	Brain brain;

	ros::Rate loop_rate(10);

	while (ros::ok()){
		ros::spinOnce();
		brain.setStateRun();
	}

	return 0;
}
