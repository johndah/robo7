#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "arduino_servo_control/SetServoAngles.h"
#include "robo7_srvs/PickupAt.h"


class PickupAtServer
{
public:

	ros::NodeHandle n;
	ros::Subscriber robot_pos_sub;
	ros::Subscriber robot_at_point_sub;
	ros::Publisher go_to_pub;
	ros::ServiceServer pickup_at_service;


	PickupAtServer()
	{
		n.param<int>("/gate_controller/r_closed", r_closed, 0);
		n.param<int>("/gate_controller/r_open", r_open, 0);
		n.param<int>("/gate_controller/l_closed", l_closed, 0);
		n.param<int>("/gate_controller/l_open", l_open, 0);
		at_point = false;

		robot_pos_sub = n.subscribe("/deadreckogning/Pos", 1, &PickupAtServer::robotPosCallback, this);
		robot_at_point_sub = n.subscribe("/robot_arrived", 1, &PickupAtServer::robotAtPointCallback, this);
		go_to_pub = n.advertise<geometry_msgs::Twist>("/destination_point", 1);
		pickup_at_service = n.advertiseService("pickup_at", &PickupAtServer::pickupSequence, this);

	}

	void robotAtPointCallback(const geometry_msgs::Twist::ConstPtr &msg)
	{
    robot_pos_x = msg->linear.x;
		robot_pos_y = msg->linear.y;
		robot_pos_tet = msg->angular.z;
	}

	void robotPosCallback(const std_msgs::Bool::ConstPtr &msg)
	{
    robot_pos_x = msg->data;
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
 		geometry_msgs::Twist obejct_pos_robot = req.object_pos;
	  ROS_INFO("Pickup sequence started");

		openGate();
		// placeholder sleep
		usleep(2*1000000);

		geometry_msgs::Twist object_pos;
		object_pos.linear.x = (obejct_pos_robot.linear.x * cos(robot_pos_tet) + obejct_pos_robot.linear.y * sin(robot_pos_tet)) + robot_pos_x;
		object_pos.linear.y = (obejct_pos_robot.linear.x * sin(robot_pos_tet) + obejct_pos_robot.linear.y * cos(robot_pos_tet)) + robot_pos_y;

		go_to_pub.publish(object_pos);

		usleep(1000000);

		while (!at_point){
			// do nothing
		}

		closeGate();
		// placeholder sleep
		usleep(2*1000000);

		res.success = true;
    return true;
	}


private:
	float robot_pos_x;
	float robot_pos_y;
	float robot_pos_tet;
	arduino_servo_control::SetServoAngles ang;
	int r_closed;
	int r_open;
	int l_closed;
	int l_open;

	bool at_point;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gate_controller_server");

	PickupAtServer gate_controller_server;

	ros::Rate loop_rate(100);

	ROS_DEBUG("Gate controller server running");

	while (gate_controller_server.n.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
