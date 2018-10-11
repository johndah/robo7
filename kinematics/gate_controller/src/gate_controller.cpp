#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "arduino_servo_control/SetServoAngles.h"

class GateControllerNode
{
public:

	ros::NodeHandle n;
	ros::Subscriber gate_state_sub;

	GateControllerNode()
	{
		gate_closed = true;

		n.param<int>("/gate_controller/r_closed", r_closed, 0);
		n.param<int>("/gate_controller/r_open", r_open, 0);
		n.param<int>("/gate_controller/l_closed", l_closed, 0);
		n.param<int>("/gate_controller/l_open", l_open, 0);

		gate_state_sub = n.subscribe("gate_closed", 1, &GateControllerNode::gateCallback, this);

	}


	void gateCallback(const std_msgs::Bool::ConstPtr &msg)
	{
    if (msg->data != gate_closed){
      toggleGate();
    }
	}


	void toggleGate()
	{
		if (gate_closed){
		// if the gate is closed, then we should open it
			ang.request.angle_servo_0 = l_open;
			ang.request.angle_servo_1 = r_open;
		}
		else
		{
		// else we should close it
			ang.request.angle_servo_0 = l_closed;
			ang.request.angle_servo_1 = r_closed;
		}

		if (ros::service::call("/arduino_servo_control/set_servo_angles", ang))
		{
				ROS_DEBUG("Gate toggled");
				gate_closed = !gate_closed; //only change state if service call was sucsessfull
		}

  }

private:
	int r_closed;
	int r_open;
	int l_closed;
	int l_open;
	bool gate_closed;
	arduino_servo_control::SetServoAngles ang;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gate_controller");

	GateControllerNode gate_controller_node;

	ros::Rate loop_rate(100);

	ROS_DEBUG("Gate controller running");

	while (gate_controller_node.n.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
