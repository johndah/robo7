#include "ros/ros.h"
#include "std_msgs"

class MotorControllerNode
{
public:

	ros::NodeHandle n;
	ros::Subscriber encoder_sub;
	ros::Subscriber twist_sub;
	ros::Publisher pwm_pub;

	MotorControllerNode()
	{

	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_controller");

}