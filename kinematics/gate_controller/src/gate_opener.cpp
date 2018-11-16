#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "arduino_servo_control/SetServoAngles.h"
#include "robo7_srvs/door_opener.h"

class openGateServer
{
public:
	ros::NodeHandle n;
	ros::ServiceServer open_door_service;

	openGateServer()
	{
		// Parameters that defines the servo angles for open and closed
		n.param<int>("/gate_controller/r_closed", r_closed, 0);
		n.param<int>("/gate_controller/r_open", r_open, 0);
		n.param<int>("/gate_controller/l_closed", l_closed, 0);
		n.param<int>("/gate_controller/l_open", l_open, 0);

		open_door_service = n.advertiseService("/gate_controller/open_door", &openGateServer::open_Sequence, this);
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

	bool open_Sequence(robo7_srvs::door_opener::Request &req,
         robo7_srvs::door_opener::Response &res)
	{
		open = req.open_door;

		if(open)
		{
			openGate();
		}
		else
		{
			closeGate();
		}

		// placeholder sleep
		usleep(2*1000000);

		res.success = true;
    return true;
	}

private:
	arduino_servo_control::SetServoAngles ang;
	int r_closed;
	int r_open;
	int l_closed;
	int l_open;
	bool open;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gate_opener_server");

	openGateServer openGateServer_;

	ros::Rate loop_rate(100);

	ROS_INFO("Gate opener server running");

	ros::spin();

	return 0;
}
