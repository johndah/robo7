#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "phidgets/motor_encoder.h"

#include <robo7_msgs/WheelAngularVelocities.h>

class MotorControllerNode
{
public:

	ros::NodeHandle n;
	ros::Subscriber l_encoder_sub;
	ros::Subscriber r_encoder_sub;
	ros::Subscriber twist_sub;
	ros::Publisher l_pwm_pub;
	ros::Publisher r_pwm_pub;

	MotorControllerNode()
	{
		counts_pr = 3591.84;

		l_encoder_sub = n.subscribe("/l_motor/encoder", 100, &MotorControllerNode::l_encoderCallback, this);
		r_encoder_sub = n.subscribe("/r_motor/encoder", 100, &MotorControllerNode::r_encoderCallback, this);
		// twist_sub = n.subscribe("ref_vels", 100, &MotorControllerNode::twistCallback, this);
		l_pwm_pub = n.advertise<std_msgs::Float32>("/l_motor/cmd_vel", 100);
		r_pwm_pub = n.advertise<std_msgs::Float32>("/r_motor/cmd_vel", 100);
	}

	void l_encoderCallback(const phidgets::motor_encoder::ConstPtr &msg)
	{
		l_delta_encoder = msg->count_change;
		l_estimated_w = l_delta_encoder / counts_pr * 3.14 * 30;

		ROS_INFO("l_delta_encoder: %d", l_delta_encoder);
		ROS_INFO("l_estimated_w: %f", l_estimated_w);
	}

	void r_encoderCallback(const phidgets::motor_encoder::ConstPtr &msg)
	{
		r_delta_encoder = msg->count_change;
		r_estimated_w = r_delta_encoder / counts_pr * 3.14 * 30;
		ROS_INFO("r_delta_encoder: %d", r_delta_encoder);
	}

	// void twistCallback(const robo7_msgs::WheelAngularVelocities::ConstPtr &msg)
	// {
	//
	// }

	void pidController()
	{

	}

private:

	int l_delta_encoder;
	int r_delta_encoder;
	float l_estimated_w;
	float r_estimated_w;

	float counts_pr;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_controller");

	MotorControllerNode motor_controller_node;

	ros::Rate loop_rate(10);

	while (motor_controller_node.n.ok())
	{
		motor_controller_node.pidController();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
