#include "ros/ros.h"
#include "iostream"
#include "fstream"
#include "std_msgs/Bool.h"
#include "robo7_msgs/aObject.h"
#include "robo7_msgs/allObjects.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"


class Brain
{
public:
	ros::NodeHandle n;
	ros::Subscriber objects_sub;
	ros::Subscriber e_break_sub;

	Brain()
	{
    n.param<std::string>("/brain/objs_file", objs_file, "objs.txt");
		carrying_object = false;
		e_break = false;
		at_home = false;
		at_object = false;

		readObjsfile();

		ROS_INFO("Read %d objects from file", read_objs.size());
	}


	bool readObjsfile(){
		std::string line;

		std::ifstream objs_fs;
		objs_fs.open(objs_file.c_str());

		if (!objs_fs.is_open()){
				ROS_WARN("Could not read objects file!");
				return false;

		} else{
			ROS_INFO("Reading objects from file");

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

					read_objs.push_back(a_object);

			}
			return true;
		}
	}


	void setBestObject()
	{
		ROS_INFO("Evaluating objects");

	}


	// void setStateRun()
	// {
	// 	ROS_INFO("Setting state");
	//
	// 	if(e_break){
	// 		state = ST_HANDLE_E_BREAK;
	// 	}
	//
	// 	// if(best_object != NULL &! carrying_object){
	// 	// 	state = ST_EVALUATE;
	// 	// }
	//
	// 	if(at_object){
	// 		state = ST_PICK_UP_OBJ;
	// 	}
	//
	// 	if (carrying_object &! at_home){
	// 		state = ST_GO_HOME
	// 	}
	//
	// 	if (carrying_object && at_home){
	// 		state = ST_DROP_OBJ
	// 	}
	//
	// }


	// void act(){
	// 	switch (mode) {
	// // EXPOLORATION MODE
	// 		case EXPLORE:
	// 		ROS_INFO("Brain is in explore-mode");
	//
	// 		switch(state) {
	// 			case ST_HANDLE_E_BREAK:
	// 				// The emergency break was triggered!
	// 				ROS_INFO("Brain sees that the ebreak is triggered")
	// 				break;
	// 		}
	// 		break;
	//
	// // RUNNING MODE
	// 		case RUN:
	// 		brain.setStateRun()
	// 		ROS_INFO("Brain is in run-mode");
	//
	// 		switch(state) {
	//
	// 			case ST_EVALUATE:
	// 				// Start the run mode
	// 				setBestObject();
	//
	// 				break;
	//
	// 			case ST_PICK_UP_OBJ:
	// 				// Pick up an object
	// 				geometry_msgs::Twist object_pos;
	// 				object_pos.linear.x = 0.1; //drive 10cm forward
	//
	// 				if (ros::service::call("/gate_controller/pickup_at", object_pos)){
	// 					carrying_object = true;
	// 					//best_object = NULL;
	// 					at_object = false;
	// 				}
	// 				else {
	// 					ROS_WARN("PICKUP FAILED!");
	// 					at_object = false;
	// 				}
	//
	// 				break;
	//
	//
	// 			case ST_DROP_OBJ:
	// 				// Drop up an object
	//
	// 				// go home after object is picked up
	// 				carrying_object = false;
	// 				break;
	//
	//
	// 			case ST_GO_HOME:
	// 				// Go home to drop of object
	// 				at_home = true;
	// 				break;
	//
	//
	// 			case ST_GO_TO_OBJECT:
	//
	// 				/* 1. Get the recorded position of the object
	// 					 2. Calculate a new position/orientation based on surrounding
	// 					 walls/obstacles for better pickup of the object
	// 					 3. Travel to the new position
	// 					*/
	//
	// 				// wait for callback from travel service
	// 				if(true){
	// 					at_object = true;
	// 				}
	//
	// 				 // Now pick up the object
	// 				break;
	//
	//
	// 			case ST_HANDLE_E_BREAK:
	// 				// The emergency break was triggered!
	// 				ROS_INFO("Brain sees that the ebreak is triggered")
	//
	// 				// BACK UP?
	//
	// 				e_break = false;
	// 				break;
	//
	// 		}
	// 		break;
	// 	}
	// }

private:
	std::string mode;
	std::string state;
	robo7_msgs::aObject all_objects[];
	robo7_msgs::aObject best_object;
	std::vector<robo7_msgs::aObject> read_objs;
	bool carrying_object;
	bool e_break;
	bool at_home;
	bool at_object;
	std::string objs_file;


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "brain");

	Brain brain;

	ros::Rate loop_rate(10);

	// TODO: CHECK TIME HERE

	//brain.act();

	ros::spin();

	return 0;
}
