//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <robo7_msgs/PWM.h>


// Control @ 10 Hz
double control_frequency = 10.0;

class deadReckogning
{
public:
  ros::NodeHandle n;
  ros::Subscriber encoder_Left;
  ros::Subscriber encoder_Right;
  ros::Subscrider pwm_Left;
  ros::Subscriber pwm_Right;
  ros::Publisher robot_position;

  //Initialisation
  float x_pos = 0;
  float y_pos = 0;
  float angle_pos = 0;

  deadReckogning()
  {
    n_ = ros::NodeHandle("~");

    encoder_Left = n.subscribe("/l_motor/encoder", 1000, &deadReckogning::encoder_L_callBack, this);
    encoder_Right = n.subscribe("/r_motor/encoder", 1000, &deadReckogning::encoder_R_callBack, this);
    pwm_Left = n.subscribe("pwm_l", 1000, &deadReckogning::pwm_L_callBack, this);
    pwm_Right = n.subscribe("pwm_r", 1000, &deadReckogning::pwm_R_callBack, this);

    robot_position = n.advertise<geometry_msgs::Twist>("deadReckogning/Pos", 1000);

  }

  void encoder_L_callBack(const ras_lab1_msgs::PWM::ConstPtr &msg)
  {
      encoder_L = msg->count_change;
  }

  void encoder_R_callBack(const ras_lab1_msgs::PWM::ConstPtr &msg)
  {
      encoder_R = msg->count_change;
  }

  void pwm_L_callBack(const ras_lab1_msgs::PWM::ConstPtr &msg)
  {
      pwm_L = msg->PWM_l;
  }

  void pwm_R_callBack(const ras_lab1_msgs::PWM::ConstPtr &msg)
  {
      pwm_R = msg->PWM_r;
  }


  void updatePosition(){
    //Generate the future published twist msg
    geometry_msgs::Twist twist_msg;

    //forward or backward
    motor_turn_L = sgn(pwm_L);
    motor_turn_R = sgn(pwm_R);
    //Guess the values of both wheel's angular speeds with signs
    om_L = angular_motor_speed(encoder_L, motor_turn_L);
    om_R = angular_motor_speed(encoder_R, motor_turn_R);
    //Compute the linear and angular velocities
    ang_vel = angular_velocity(om_L, om_R);
    lin_vel = linear_velocity(om_L, om_R);

    //Update the position and orientation of the robot
    x_pos = x_pos + (lin_vel*Dt) * cos(angle_pos);
    y_pos = y_pos + (lin_vel*Dt) * sin(angle_pos);
    angle_pos = angle_pos + (ang_vel*Dt);

    twist_msg.linear.x = x_pos;
    twist_msg.linear.y = y_pos;
    twist_msg.linear.z = 0.0;

    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = angle_pos;
  }


private:
  //All the physical dimensions of the robot
  float wheel_radius = 98/2; //mm
  float wheel_distance = 219.8; //mm
  float tics_per_rev = 3591.84;
  float pi = 3.14159265358979323846;

  //Other parameters
  float Dt = 1/control_frequency; //ms - time between two consecutive iterations


  //encoders values
  int encoder_L = 0;
  int encoder_R = 0;
  //PWM values -> in order to know how it rotate
  int pwm_L = 0;
  int pwm_R = 0;
  //forward or backward
  int motor_turn_L;
  int motor_turn_R;
  //Guess the values of both wheel's angular speeds with signs
  float om_L;
  float om_R;
  //Compute the linear and angular velocities
  float ang_vel;
  float lin_vel;

  //Other useful function
  int sgn(int v) {
    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;
  }

  float angular_motor_speed(int encod, int motor_turn) {
    return (2*pi/tics_per_rev)*encod;
  }

  float linear_velocity(float left_wheel_speed, float right_wheel_speed) {
    return wheel_radius*(right_wheel_speed + left_wheel_speed)/2;
  }

  float angular_velocity(float left_wheel_speed, float right_wheel_speed) {
    return wheel_radius*(right_wheel_speed - left_wheel_speed)/wheel_distance;
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dead_reckogning");

    deadReckogning dead_reckogning;

    ros::Rate loop_rate(control_frequency);

    while(dead_reckogning.n.ok())
    {
        dead_reckogning.updatePosition();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
