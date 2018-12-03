#include "ros/ros.h"
#include "iostream"
#include "fstream"
#include "std_msgs/Bool.h"
#include "robo7_msgs/aObject.h"
#include "robo7_msgs/allObjects.h"
#include "robo7_msgs/the_robot_position.h"
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
	ros::Subscriber robo_pos_sub;
	ros::Publisher all_obj_pub;
	ros::ServiceClient distance_srv;
	ros::ServiceClient occupancy_srv;
	ros::ServiceClient go_to_srv;
	ros::ServiceClient pickup_at_srv;
	static int objs_values[];


	Brain()
	{
		n.param<int>("/brain/weight_thresh", weight_thresh, 5);
		n.param<float>("/brain/occu_thresh", occu_thresh, 0.4);
		n.param<float>("/brain/robot_pose_dist", robot_pose_dist, 0.25); // distance avay from the robot center to search for a pose
		n.param<float>("/brain/robot_pose_object_delta", robot_pose_object_delta, 0.09);  // compensate for the "cave" not beeing at robot center
    n.param<std::string>("/brain/objs_file", objs_file, "objs.txt");

		distance_srv = n.serviceClient<robo7_srvs::distanceTo>("/distance_grid/distance");
		occupancy_srv = n.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
		go_to_srv = n.serviceClient<robo7_srvs::GoTo>("/kinematics/go_to");
		pickup_at_srv = n.serviceClient<robo7_srvs::PickupAt>("/gate_controller/pickup_at");

		robo_pos_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &Brain::roboPosCallback, this);

		all_obj_pub = n.advertise<robo7_msgs::allObjects>("/vision/all_objects", 1);


		robot_position_set = false;
		go_to_pose.linear.x = -1.0;

		home_pose.linear.x = 0.215;
		home_pose.linear.y = 0.2;
		home_pose.angular.z = -1.57;

		stop_seq = false;
		got_obj_dest = false;
		carrying_object = false;
		e_break = false;
		at_home = false;
		at_object = false;

		if (!readObjsfile()){
			stop_seq = true;
		}

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


	void publishObjects(){
		if(read_objs.size() > 0){
			ROS_INFO("Brain: Publishing objects for visualization");
			robo7_msgs::allObjects pub_obj;
			pub_obj.objects = read_objs;
			all_obj_pub.publish(pub_obj);
		}
	}


	void roboPosCallback(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    robo_pos = msg->position;
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
  	return (ob1.total_votes > ob2.total_votes);
	}


	int getDistance(float x, float y, bool reverse){
		ros::spinOnce();

		robo7_srvs::distanceTo::Request srv_req;
		robo7_srvs::distanceTo::Response srv_resp;

		if (!robot_position_set){
			ROS_WARN("Brain: Trying to evaluate distance to objects without knowing the robot position");
			return -1;
		}

		if (!reverse){
			srv_req.x_from = robo_pos.linear.x;
			srv_req.y_from = robo_pos.linear.y;
			srv_req.x_to = x;
			srv_req.y_to = y;

		} else{
			srv_req.x_from = x;
			srv_req.y_from = y;
			srv_req.x_to = robo_pos.linear.x;
			srv_req.y_to = robo_pos.linear.y;
		}

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

		srv_req.destination_pose = to_pose;

		go_to_srv.call(srv_req, srv_resp);

		return srv_resp.success;
	}


	bool callPickupAt( float drive_dist, bool drop_mode) {
		robo7_srvs::PickupAt::Request srv_req;
		robo7_srvs::PickupAt::Response srv_resp;

		srv_req.dist = drive_dist;
		srv_req.drop = drop_mode;

		pickup_at_srv.call(srv_req, srv_resp);

		return srv_resp.success;
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

		//ROS_INFO("ct class: %d", best_object.obj_class);
	}


	geometry_msgs::Twist getDestPose(float x_dest, float y_dest) {
		geometry_msgs::Twist pose;
		pose.linear.x = -1;

		float lowest_occupancy = 1.0;
		int lowest_dist = 1000000;

		float test_lowest_occu_x;
		float test_lowest_occu_y;
		float test_lowest_occu_ang_z;

		// returns a pose that is possible to travel to, given the destination
		for (float alp = 0 ; alp < 2*pi ; alp = alp+(pi/8)){

			float test_x = x_dest + (robot_pose_dist * cos(alp));
			float test_y = y_dest + (robot_pose_dist * sin(alp));

			float new_occupancy = getOccupancy(test_x, test_y);
			int new_distance = getDistance(test_x, test_y, true);

			//ROS_INFO("Brain:getDestPose testing: x:%f, y:%f and got got occupancy: %f", test_x, test_y, new_occupancy);
			//ROS_INFO("Brain: Occ: %f, Dist: %d", (float)new_occupancy, (int)new_distance);

			if(new_occupancy < occu_thresh && new_distance < lowest_dist){
				//ROS_INFO("Brain: FOUND A BETTER POSE");
				lowest_dist = new_distance;
				pose.linear.x = test_x;
				pose.linear.y = test_y;
				pose.angular.z = std::fmod((alp + pi)+pi, 2*pi) - pi;

			}

			if(new_occupancy < lowest_occupancy){
				lowest_occupancy = new_occupancy;
				test_lowest_occu_x = test_x;
				test_lowest_occu_y = test_y;
				test_lowest_occu_ang_z = std::fmod((alp + pi)+pi, 2*pi) - pi;

			}

		}

		if (lowest_dist == 1000000){
			// in case no object fulfills the threshold, take the best of the worst
			ROS_WARN("Brain: No distance was found for the testing angles around the object");
			pose.linear.x = test_lowest_occu_x;
			pose.linear.y = test_lowest_occu_y;
			pose.angular.z = test_lowest_occu_ang_z;
		}

		return pose;

	}


	void setStateRun(){
		//ROS_INFO("Brain: setting state");

		if(!got_obj_dest &! carrying_object){
		 	state = "ST_EVALUATE";
		}

		if(got_obj_dest &! at_object &! carrying_object){
		 	state = "ST_GO_TO_OBJECT";
		}

		if(at_object &! carrying_object){
			state = "ST_PICK_UP_OBJ";
		}

		if (carrying_object &! at_home){
			state = "ST_GO_HOME";
		}

		if (carrying_object && at_home){
			state = "ST_DROP_OBJ";
		}

		if (stop_seq){
			// when all is done or something is seriously wrong
			state = "ST_STOP_SEQ";
		}
		act();

	}


	void act(){
			if (state == "ST_EVALUATE"){
				ROS_INFO("Brain: ST_EVALUATE");
				ROS_INFO("Brain: Go to pose x: %f", go_to_pose.linear.x);

				if (read_objs.size() > 0){
					while (go_to_pose.linear.x == -1.0 && read_objs.size() > 0){
						publishObjects();

						ROS_WARN("Brain: %d objects left to pick up.", (int)read_objs.size());

						setBestObject();

						go_to_pose = getDestPose(best_object.pos.x, best_object.pos.y);

						//ROS_INFO("Brain: Best object at : x: %f y: %f", best_object.pos.x, best_object.pos.y);
						//ROS_INFO("Brain: Go to pose for that object: : x: %f y: %f ang.z: %f", go_to_pose.linear.x, go_to_pose.linear.y, go_to_pose.angular.z);

					}

					if(go_to_pose.linear.x == -1){
						ROS_WARN("Brain: Failed to find robot pose for object, stopping sequence");
						stop_seq = true;
					} else{
						got_obj_dest = true;
					}


				} else{
					ROS_WARN("Brain: No more objects, stopping sequence");
					stop_seq = true;
				}

			}

			if (state == "ST_GO_TO_OBJECT"){
				ROS_INFO("Brain: ST_GO_TO_OBJECT");

				if(callGoTo(go_to_pose)){
					at_object = true;
					go_to_pose.linear.x = -1.0;
				} else{
					stop_seq = true;
					ROS_WARN("Brain: we did not reach the object, stopping sequence");
				}

			}

			if (state == "ST_PICK_UP_OBJ"){
				ROS_INFO("Brain: ST_PICK_UP_OBJ");

				callPickupAt((robot_pose_dist - robot_pose_object_delta), false);

				publishObjects();

				carrying_object = true;
				at_object = false;
				got_obj_dest = false;

			}


			if (state == "ST_DROP_OBJ"){
				ROS_INFO("Brain: ST_DROP_OBJ");

				callPickupAt((robot_pose_dist - robot_pose_object_delta), true);

				carrying_object = false;
				at_home = false;
		  }


			if (state == "ST_GO_HOME"){
				ROS_INFO("Brain: ST_GO_HOME");

				if(callGoTo(home_pose)){
					at_home = true;
				} else{
					stop_seq = true;
					ROS_WARN("Brain: we did not reach home, stopping sequence");
				}

			}

			if (state == "ST_STOP_SEQ"){
				ROS_INFO("Brain: ST_STOP_SEQ");
				ROS_WARN("Brain: STOPPING");

				publishObjects();

				return;

			}
			ros::spinOnce();
			setStateRun();

	}

private:
	std::string state;
	geometry_msgs::Twist robo_pos;
	geometry_msgs::Twist home_pose;
	geometry_msgs::Twist go_to_pose;
	robo7_msgs::aObject best_object;
	std::vector<robo7_msgs::aObject> read_objs;
	bool robot_position_set;
	bool carrying_object;
	bool got_obj_dest;
	bool e_break;
	bool at_home;
	bool at_object;
	bool stop_seq;
	int weight_thresh;
	float occu_thresh;
	std::string objs_file;
	float robot_pose_dist;
	float robot_pose_object_delta;


};

int Brain::objs_values[14] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "brain");

	Brain brain;

	ros::Rate loop_rate(0.3);

	loop_rate.sleep();

	brain.setStateRun();


	// while (ros::ok()){
	// 	ros::spinOnce();
	// 	brain.setStateRun();
	// }

	return 0;
}
