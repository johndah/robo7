#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "arduino_servo_control/SetServoAngles.h"
#include "robo7_srvs/PickupAt.h"
#include "robo7_srvs/MoveStraight.h"

class PickupAtServer
{
public:
	ros::NodeHandle n;
	ros::ServiceServer pickup_at_srv_server;
	ros::ServiceClient move_straight_srv;

	PickupAtServer()
	{
		// Parameters that defines the servo angles for open and closed
		n.param<int>("/gate_controller/r_closed", r_closed, 0);
		n.param<int>("/gate_controller/r_open", r_open, 0);
		n.param<int>("/gate_controller/l_closed", l_closed, 0);
		n.param<int>("/gate_controller/l_open", l_open, 0);

		pickup_at_srv_server = n.advertiseService("/gate_controller/pickup_at", &PickupAtServer::pickupSequence, this);

		move_straight_srv = n.serviceClient<robo7_srvs::MoveStraight>("/kinematics/path_follower/straight_move");

	}


	void openGate() {
		ang.request.angle_servo_0 = l_open;
		ang.request.angle_servo_1 = r_open;

		if (ros::service::call("/arduino_servo_control/set_servo_angles", ang)) {
				ROS_DEBUG("Gate opened");
		}

  }


	void closeGate() {
		ang.request.angle_servo_0 = l_closed;
		ang.request.angle_servo_1 = r_closed;

		if (ros::service::call("/arduino_servo_control/set_servo_angles", ang)) {
				ROS_DEBUG("Gate closed");
		}

	}


	bool pickupSequence(robo7_srvs::PickupAt::Request &req,
         robo7_srvs::PickupAt::Response &res)
	{

		float travel_dist = req.dist;
		bool drop_mode = req.drop;

		robo7_srvs::MoveStraight::Request srv_move_straight_req;
		robo7_srvs::MoveStraight::Response srv_move_straight_resp;

		srv_move_straight_req.desired_distance = travel_dist;
		srv_move_straight_req.move_backward = drop_mode;

		ROS_INFO("Gate Controller: Opening gate");

		openGate();
		usleep(2*1000000);

		if (!drop_mode){
			ROS_INFO("Gate Controller: moving forward for %fm", travel_dist);
		} else{
			ROS_INFO("Gate Controller: moving backward for %fm", travel_dist);
		}

		move_straight_srv.call(srv_move_straight_req, srv_move_straight_resp);

		closeGate();
		usleep(2*1000000);

		res.success = true;

		ROS_INFO("Gate Controller: Done");
    return true;
	}

private:
	arduino_servo_control::SetServoAngles ang;
	int r_closed;
	int r_open;
	int l_closed;
	int l_open;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gate_controller_server");

	PickupAtServer gate_controller_server;

	ros::Rate loop_rate(100);

	ROS_INFO("Gate controller server running");

	ros::spin();

	return 0;
}
