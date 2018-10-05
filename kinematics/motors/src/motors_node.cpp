

#include <ros/ros.h>
#include <robo7_msgs/PWM.h>
#include <robo7_msgs/Encoders.h>
#include <robo7/kinematics/motors.h>

#include <geometry_msgs/Twist.h>



class KobukiMotorsNode
{
public:

    ros::NodeHandle n_;
    ros::Subscriber pwm_subscriber_;
    ros::Publisher encoders_publisher_;
    ros::Publisher twist_publisher_;

    KobukiMotorsNode()
    {
        n_ = ros::NodeHandle("~");
        kobuki_motors_ = new KobukiMotors();

        pwm_ = std::vector<int>(2, 0);
        t_pwm_ = ros::Time::now();

        wheel_radius_ = 0.0352;
        base_ = 0.23;

        pwm_subscriber_ = n_.subscribe("pwm", 1, &KobukiMotorsNode::pwmCallback, this);
        encoders_publisher_ = n_.advertise<ras_lab1_msgs::Encoders>("encoders", 1);
        twist_publisher_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    }

    ~KobukiMotorsNode()
    {
        delete kobuki_motors_;
    }


    // [0] corresponds to left wheel, [1] corresponds to right wheel
    void pwmCallback(const ras_lab1_msgs::PWM::ConstPtr &msg)
    {
        pwm_[0] = msg->PWM1;
        pwm_[1] = msg->PWM2;
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
        encoders_msg.encoder1 = abs_encoders[0];
        encoders_msg.encoder2 = abs_encoders[1];

        encoders_msg.delta_encoder1 = diff_encoders[0];
        encoders_msg.delta_encoder2 = diff_encoders[1];


        encoders_publisher_.publish(encoders_msg);


        // calculate kinematics and send twist to robot simulation node
        geometry_msgs::Twist twist_msg;

        double linear_vel = (wheel_angular_velocities[1] + wheel_angular_velocities[0])*0.5*wheel_radius_;
        double angular_vel = (wheel_angular_velocities[1] - wheel_angular_velocities[0])*wheel_radius_/base_;

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
