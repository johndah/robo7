#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"

class Speaker
{
  public:
	ros::NodeHandle n;
	ros::Subscriber object_sub;
  ros::Publisher espeak_pub;

  std::string obj_classes[14] = {
        "a yellow ball",
        "a yellow cube",
        "a green cube",
        "a green cylinder",
        "a green hollow cube",
        "a orange cross",
        "patric",
        "a red cylinder",
        "a red hollow cube",
        "a red ball",
        "a blue cube",
        "a blue triangle",
        "a purple cross",
        "a purple star"};

	Speaker()
	{
		object_sub = n.subscribe("/vision/object/class", 1, &Speaker::newObjectCallback, this);
    espeak_pub = n.advertise<std_msgs::String>("/espeak/string", 1);
	}

	void newObjectCallback(const std_msgs::Int16::ConstPtr &msg)
	{
		int obj = msg->data;

    std_msgs::String msg_out;
    std::stringstream ss;

    if(0 <= obj && obj < 14){
      ss << "I see " << obj_classes[obj];
    }
    else{
      ss << "I am confused";
    }

    msg_out.data = ss.str();
    espeak_pub.publish(msg_out);
    // ROS_INFO("%s", msg_out.data.c_str());
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "speaker");

	Speaker speaker;

	ros::Rate loop_rate(10);

	ROS_INFO("Speaker node is running");

	ros::spin();

	return 0;
}
