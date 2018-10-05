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
		// counts_pr = 3591.84;
		// dt = 0.1;

		int_error = std::vector<float>(2, 0);

		l_encoder_sub = n.subscribe("/l_motor/encoder", 100, &MotorControllerNode::l_encoderCallback, this);
		r_encoder_sub = n.subscribe("/r_motor/encoder", 100, &MotorControllerNode::r_encoderCallback, this);
		// twist_sub = n.subscribe("ref_vels", 100, &MotorControllerNode::twistCallback, this);
		l_pwm_pub = n.advertise<std_msgs::Float32>("/l_motor/cmd_vel", 100);
		r_pwm_pub = n.advertise<std_msgs::Float32>("/r_motor/cmd_vel", 100);
	}

	void l_encoderCallback(const phidgets::motor_encoder::ConstPtr &msg)
	{
		delta_encoder[0] = msg->count_change;
		estimated_w[0] = delta_encoder[0] / counts_pr * 3.14 * 30;

		ROS_INFO("l_delta_encoder: %d", delta_encoder[0]);
		ROS_INFO("l_estimated_w: %f", estimated_w[0]);
	}

	void r_encoderCallback(const phidgets::motor_encoder::ConstPtr &msg)
	{
		delta_encoder[1] = msg->count_change;
		estimated_w[1] = delta_encoder[1] / counts_pr * 3.14 * 30;
		ROS_INFO("r_delta_encoder: %d", delta_encoder[1]);
	}

	void twistCallback(const robo7_msgs::WheelAngularVelocities::ConstPtr &msg)
	{
		desired_w[0] = msg->w_l;
		desired_w[1] = msg->w_r;
	}

	void pidController()
	{

		std_msgs::Float32 l_pwm_msg;
		std_msgs::Float32 r_pwm_msg;

		error[0] = desired_w[0] - estimated_w[0];
		error[1] = desired_w[1] - estimated_w[1];

		int_error[0] = int_error[0] + error[0] * dt;
		int_error[1] = int_error[1] + error[1] * dt;

		l_pwm_msg = P * error[0] + I * int_error[0] + D * error[0] / dt;
		r_pwm_msg = P * error[1] + I * int_error[1] + D * error[1] / dt;

		l_pwm_pub.publish(l_pwm_msg);
		r_pwm_pub.publish(r_pwm_msg); 
	}

private:

	std::vector<int> delta_encoder;
	std::vector<float> estimated_w;
	std::vector<float> desired_w;
	std::vector<float> int_error;

	std::vector<float> P = {1.0, 1.0};
	std::vector<float> I = {1.0, 1.0};
	std::vector<float> D = {1.0, 1.0};
	float dt = 0.1;
	float counts_pr = 3591.84;

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
