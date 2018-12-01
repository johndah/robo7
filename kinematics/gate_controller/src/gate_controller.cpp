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
	ros::Subscriber robot_pos_sub;
	ros::Subscriber robot_at_point_sub;
	ros::Publisher go_to_pub;
	ros::ServiceServer pickup_at_srv_server;
	ros::ServiceClient move_straight_srv;


	PickupAtServer()
	{
		// Parameters that defines the servo angles for open and closed
		n.param<int>("/gate_controller/r_closed", r_closed, 0);
		n.param<int>("/gate_controller/r_open", r_open, 0);
		n.param<int>("/gate_controller/l_closed", l_closed, 0);
		n.param<int>("/gate_controller/l_open", l_open, 0);
		at_point = false;

		robot_pos_sub = n.subscribe("/dead_reckoning/Pos", 1, &PickupAtServer::robotPosCallback, this);
		robot_at_point_sub = n.subscribe("/robot_arrived", 1, &PickupAtServer::robotAtPointCallback, this);
		go_to_pub = n.advertise<geometry_msgs::Twist>("/destination_point", 1);
		pickup_at_srv_server = n.advertiseService("/gate_controller/pickup_at", &PickupAtServer::pickupSequence, this);

		move_straight_srv = n.serviceClient<robo7_srvs::MoveStraight>("/kinematics/path_follower/straight_move");


	}

	void robotPosCallback(const geometry_msgs::Twist::ConstPtr &msg)
	{
    robot_pos_x = msg->linear.x;
		robot_pos_y = msg->linear.y;
		robot_pos_tet = msg->angular.z;
	}

	void robotAtPointCallback(const std_msgs::Bool::ConstPtr &msg)
	{
    at_point = msg->data;
	}

	void openGate()
	{
		ang.request.angle_servo_0 = l_open;
		ang.request.angle_servo_1 = r_open;

		if (ros::service::call("/arduino_servo_control/set_servo_angles", ang))
		{
				ROS_DEBUG("Gate opened");
		}
  }

	void closeGate()
	{
		ang.request.angle_servo_0 = l_closed;
		ang.request.angle_servo_1 = r_closed;

		if (ros::service::call("/arduino_servo_control/set_servo_angles", ang))
		{
				ROS_DEBUG("Gate closed");
		}
	}

	bool pickupSequence(robo7_srvs::PickupAt::Request &req,
         robo7_srvs::PickupAt::Response &res)
	{
		at_point = false;

	  ROS_DEBUG("Pickup sequence started");

		float travel_dist = req.dist;
		bool drop_mode = req.drop;

		openGate();

		robo7_srvs::MoveStraight::Request srv_move_straight_req;
		robo7_srvs::MoveStraight::Response srv_move_straight_resp;

		srv_move_straight_req.desired_distance = travel_dist;
		srv_move_straight_req.desired_distance = drop_mode;

		move_straight_srv.call(srv_move_straight_req, srv_move_straight_resp);

		//usleep(1000000);
		
		closeGate();

		// placeholder sleep
		usleep(2*1000000);

		res.success = true;

		ROS_DEBUG("Pickup sequence done");
    return true;
	}

private:
	arduino_servo_control::SetServoAngles ang;
	int r_closed;
	int r_open;
	int l_closed;
	int l_open;
	float robot_pos_x;
	float robot_pos_y;
	float robot_pos_tet;
	bool at_point;

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
