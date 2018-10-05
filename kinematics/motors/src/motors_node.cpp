#include <ros/ros.h>
#include <std_msgs/Float32.h>

//ros::Subscriber pwm_subscriber_;
//ros::Publisher encoders_publisher_;
//ros::Publisher publisher_velocity;

//rostopic pub -r 10 l_motor/cmd_vel std_msgs/Float32 "data: -10"


//pwm_subscriber_ = n_.subscribe("pwm", 1, &MotorsNode::pwmCallback, this);
//encoders_publisher_ = n_.advertise<robo7_msgs::Encoders>("encoders", 1);

ros::Publisher publisher_velocity;

std_msgs::Float32 hej;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motors_node");

    ros::NodeHandle nh;


    publisher_velocity = nh.advertise<std_msgs::Float32>("l_motor/cmd_vel", 100);

    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Ran Encoder");
    hej.data = 20.0;


    while(ros::ok())
    {
        publisher_velocity.publish(hej);


        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}

//hej = 20.0;

//publisher_velocity.publish(hej);


/*
<node pkg="phidgets" type="motor" name="l_motor"
      args="_serial:=469412 __name:=l_motor _name:=l_motor"/>


#include <ros/ros.h>
#include <robo7_msgs/PWM.h>
#include <robo7_msgs/Encoders.h>
#include <robo7/kinematics/motors.h>

#include <geometry_msgs/Twist.h>



class MotorsNode
{
public:

    ros::NodeHandle n_;
    ros::Subscriber pwm_subscriber_;
    ros::Publisher encoders_publisher_;
    ros::Publisher twist_publisher_;

    MotorsNode()
    {
        n_ = ros::NodeHandle("~");
        motor_node = new MotorsNode();

        pwm_ = std::vector<int>(2, 0);
        t_pwm_ = ros::Time::now();

        wheel_radius_ = 0.0352;
        base_ = 0.23;

        pwm_subscriber_ = n_.subscribe("pwm", 1, &MotorsNode::pwmCallback, this);
        encoders_publisher_ = n_.advertise<robo7_msgs::Encoders>("encoders", 1);
        twist_publisher_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    }

    ~KobukiMotorsNode()
    {
        delete motor_node;
    }


    // [0] corresponds to left wheel, [1] corresponds to right wheel
    void pwmCallback(const robo7::PWM::ConstPtr &msg)
    {
        pwm_[0] = msg->PWM_l;
        pwm_[1] = msg->PWM2_r;
        t_pwm_ = ros::Time::now();
    }

    void updateMotors()
    {

        ras_lab1_msgs::Encoders encoders_msg;

        // [0] corresponds to left wheel, [1] corresponds to right wheel
        std::vector<double> wheel_angular_velocities(2, 0.0);
        std::vector<int> abs_encoders(2, 0);
        std::vector<int> diff_encoders(2, 0);


        // if more than 2 seconds have passed and no messages have been received,
        // shutdown the motors
        if((ros::Time::now()-t_pwm_).toSec()>2.0)
        {
            pwm_[0] = 0;
            pwm_[1] = 0;
        }

        kobuki_motors_->update(pwm_, wheel_angular_velocities,
                               abs_encoders, diff_encoders);


        // publish encoders
        encoders_msg.encoder_l = abs_encoders[0];
        encoders_msg.encoder_r = abs_encoders[1];

        encoders_msg.delta_encoder_l = diff_encoders[0];
        encoders_msg.delta_encoder_r = diff_encoders[1];


        encoders_publisher_.publish(encoders_msg);


        // calculate kinematics and send twist to robot simulation node
        geometry_msgs::Twist twist_msg;

        '''
        '''
        double linear_vel = (wheel_angular_velocities[1] + wheel_angular_velocities[0])*0.5*wheel_radius_;
        double angular_vel = (wheel_angular_velocities[1] - wheel_angular_velocities[0])*wheel_radius_/base_;
        '''
        '''

        double linear_vel = 1;
        double angular_vel = 0;

        twist_msg.linear.x = linear_vel;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_vel;

        twist_publisher_.publish(twist_msg);

    }

private:
    KobukiMotors *kobuki_motors_;

    // [0] corresponds to left wheel, [1] corresponds to right wheel
    std::vector<int> pwm_;
    ros::Time t_pwm_;

    double wheel_radius_;
    double base_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kobuki_motors");

    KobukiMotorsNode kobuki_motors_node;

    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);

    while(kobuki_motors_node.n_.ok())
    {
        kobuki_motors_node.updateMotors();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

*/
