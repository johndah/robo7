#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "phidgets/motor_encoder.h"

#include <robo7_msgs/WheelAngularVelocities.h>

class MotorControllerNode
{
public:

	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::Subscriber l_encoder_sub;
	ros::Subscriber r_encoder_sub;
	ros::Subscriber twist_sub;
	ros::Publisher l_pwm_pub;
	ros::Publisher r_pwm_pub;
	ros::Publisher est_L_vel;
	ros::Publisher est_R_vel;

	MotorControllerNode()
	{
		counts_pr = 897.96;
		control_freq = 100;
		dt = 1/control_freq;
		delta_encoder = std::vector<int>(2, 0);
		estimated_w = std::vector<float>(2, 0.0);
		desired_w = std::vector<float>(2, 0.0);
		error = std::vector<float>(2, 0.0);
		int_error = std::vector<float>(2, 0.0);
		dif_error = std::vector<float>(2, 0.0);
		P = std::vector<float>(2, 0);

		I = std::vector<float>(2, 0);
		D = std::vector<float>(2, 0);

		nh.param<float>("/motor_controller/l_P", P[0], 0);
		nh.param<float>("/motor_controller/r_P", P[1], 0);
		nh.param<float>("/motor_controller/l_I", I[0], 0);
		nh.param<float>("/motor_controller/r_I", I[1], 0);
		nh.param<float>("/motor_controller/l_D", D[0], 0);
		nh.param<float>("/motor_controller/r_D", D[1], 0);

		// P[0] = 6;
		// P[1] = 6;
		// I[0] = 3;
		// I[1] = 3;
		// D[0] = 0.1;
		// D[1] = 0.1;

		l_encoder_sub = n.subscribe("/l_motor/encoder", 100, &MotorControllerNode::l_encoderCallback, this);
		r_encoder_sub = n.subscribe("/r_motor/encoder", 100, &MotorControllerNode::r_encoderCallback, this);
		twist_sub = n.subscribe("ref_vels", 1, &MotorControllerNode::twistCallback, this);
		l_pwm_pub = n.advertise<std_msgs::Float32>("/l_motor/cmd_vel", 100);
		r_pwm_pub = n.advertise<std_msgs::Float32>("/r_motor/cmd_vel", 100);
		est_L_vel = n.advertise<std_msgs::Float32>("/l_motor/estimated_vel", 100);
		est_R_vel = n.advertise<std_msgs::Float32>("/r_motor/estimated_vel", 100);
	}

	void l_encoderCallback(const phidgets::motor_encoder::ConstPtr &msg)
	{
		delta_encoder[0] = msg->count_change;
		estimated_w[0] = delta_encoder[0] * 2 * 3.14 * 121 / counts_pr ;

		// ROS_INFO("l_delta_encoder: %d", delta_encoder[0]);
		ROS_INFO("Left estimated_w: %f", estimated_w[0]);
	}

	void r_encoderCallback(const phidgets::motor_encoder::ConstPtr &msg)
	{
		delta_encoder[1] = msg->count_change;
		estimated_w[1] = delta_encoder[1] * 2 * 3.14 * 121 / counts_pr;
		// ROS_INFO("r_delta_encoder: %d", delta_encoder[1]);
		ROS_INFO("Right estimated_w: %f", estimated_w[1]);
	}

	void twistCallback(const robo7_msgs::WheelAngularVelocities::ConstPtr &msg)
	{
		desired_w[0] = msg->W_l;
		desired_w[1] = -1 * msg->W_r;
	}

	void pidController()
	{

		std_msgs::Float32 l_pwm_msg;
		std_msgs::Float32 r_pwm_msg;
		std_msgs::Float32 estima_L_vel;
		std_msgs::Float32 estima_R_vel;


		dif_error[0] = desired_w[0] - estimated_w[0] - error[0];
		dif_error[1] = desired_w[1] - estimated_w[1] - error[1];

		error[0] = desired_w[0] - estimated_w[0];
		error[1] = desired_w[1] - estimated_w[1];

		estima_L_vel.data = estimated_w[0];
		estima_R_vel.data = estimated_w[1];

		int_error[0] = int_error[0] + error[0] * dt;
		int_error[1] = int_error[1] + error[1] * dt;

		l_pwm_msg.data = (P[0] * error[0] + I[0] * int_error[0] + D[0] * dif_error[0] / dt);
		r_pwm_msg.data = (P[1] * error[1] + I[1] * int_error[1] + D[1] * dif_error[1] / dt);

		l_pwm_msg.data = l_pwm_msg.data * max_motor_input / max_speed;
		r_pwm_msg.data = r_pwm_msg.data * max_motor_input / max_speed;

		if (l_pwm_msg.data > max_motor_input)
		{l_pwm_msg.data = max_motor_input;}
		else if (l_pwm_msg.data < -max_motor_input)
		{l_pwm_msg.data = -max_motor_input;}
		if (r_pwm_msg.data > max_motor_input)
		{r_pwm_msg.data = max_motor_input;}
		else if (r_pwm_msg.data < -max_motor_input)
		{r_pwm_msg.data = -max_motor_input;}

		ROS_INFO("Left pwm: %f", l_pwm_msg.data);
		ROS_INFO("Right pwm: %f", r_pwm_msg.data);
		ROS_INFO("Error : %f", error[0]);

		l_pwm_pub.publish(l_pwm_msg);
		r_pwm_pub.publish(r_pwm_msg);
		est_L_vel.publish(estima_L_vel);
		est_R_vel.publish(estima_R_vel);
	}

private:

	std::vector<int> delta_encoder;
	std::vector<float> estimated_w;
	std::vector<float> desired_w;
	std::vector<float> error;
	std::vector<float> int_error;
	std::vector<float> dif_error;

	std::vector<float> P;
	std::vector<float> I;
	std::vector<float> D;
	float control_freq;
	float dt;
	float counts_pr;

	float max_speed = 25.387;
	float max_motor_input = 100;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_controller");

	MotorControllerNode motor_controller_node;

	ros::Rate loop_rate(100);

	while (motor_controller_node.n.ok())
	{
		motor_controller_node.pidController();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
