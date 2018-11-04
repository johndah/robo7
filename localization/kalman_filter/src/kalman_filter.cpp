//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>

#include <robo7_msgs/MeasureFeedback.h>
#include <robo7_msgs/MeasureRequest.h>


// Control @ 10 Hz
double control_frequency = 100.0;

class kalmanFilter
{
public:
  ros::NodeHandle n;
  ros::Subscriber encoder_Left;
  ros::Subscriber encoder_Right;
  ros::Subscriber measure_sub;
  ros::Subscriber scan_sub;
  ros::Publisher robot_position;
  ros::Publisher new_measure_req_pub;

  kalmanFilter()
  {
    ROS_INFO("Starting EKF");
    //Initialisation of the position
    n.param<float>("/kalman_filter/initial_x_pos", x_pos, 0);
    n.param<float>("/kalman_filter/initial_y_pos", y_pos, 0);
    n.param<float>("/kalman_filter/initial_z_angle", z_angle, 0);

    //Definition of the adjustment parameters
    n.param<float>("/kalman_filter/linear_adjustment", linear_adjustment, 0);
    n.param<float>("/kalman_filter/angular_adjustment", angular_adjustment, 0);
    n.param<bool>("/kalman_filter/use_measure", use_measure, false);
    n.param<int>("/kalman_filter/time_between_two_measure", time_threshold, 1);

    //The errors values
    n.param<float>("/kalman_filter/sigma_angle", sigma_a, 0.1);
    n.param<float>("/kalman_filter/sigma_distance", sigma_d, 0.05);

    wheel_radius = 97.6/2000.0; //m
    wheel_distance = 217.3/1000.0; //m
    tics_per_rev = 897.96;
    pi = 3.14159265358979323846;

    //Initialisation
    encoder_R = 0;
    encoder_L = 0;
    count_L = 0;
    count_R = 0;
    prev_count_L = 0;
    prev_count_R = 0;

    //Other parameters
    Dt = 1/control_frequency; //ms - time between two consecutive iterations
    prev_mes_time = ros::Time::now().toSec();

    //Measure Request/Response initialisations
    request_id = 0;
    previous_measure_received = true;
    new_measure_received = false;

    //Initialize the different matrices
    reinitialize_A_W_matrices();

    encoder_Left = n.subscribe("/l_motor/encoder", 1, &kalmanFilter::encoder_L_callBack, this);
    encoder_Right = n.subscribe("/r_motor/encoder", 1, &kalmanFilter::encoder_R_callBack, this);
    measure_sub = n.subscribe("/localization/meas_update/measure_response", 1, &kalmanFilter::measureFeedback_callBack, this);
    scan_sub = n.subscribe("/scan", 1, &kalmanFilter::scan_callBack, this);
    // robot_position = n.advertise<geometry_msgs::Twist>("/localization/kalman_filter/pos", 10);
    robot_position = n.advertise<geometry_msgs::Twist>("/localization/kalman_filter/position", 10);
    new_measure_req_pub = n.advertise<robo7_msgs::MeasureRequest>("/localization/kalman_filter/measure_request", 10);

    ROS_INFO("EKF initialisation done");
  }

  void encoder_L_callBack(const phidgets::motor_encoder::ConstPtr &msg)
  {
      count_L = msg->count;
  }

  void encoder_R_callBack(const phidgets::motor_encoder::ConstPtr &msg)
  {
      count_R = msg->count;
  }

  void scan_callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
      the_lidar_scan.header = msg->header;
      the_lidar_scan.angle_min = msg->angle_min;
      the_lidar_scan.angle_max = msg->angle_max;
      the_lidar_scan.angle_increment = msg->angle_increment;
      the_lidar_scan.time_increment = msg->time_increment;
      the_lidar_scan.scan_time = msg->scan_time;
      the_lidar_scan.range_min = msg->range_min;
      the_lidar_scan.range_max = msg->range_max;
      the_lidar_scan.ranges = msg->ranges;
      the_lidar_scan.intensities = msg->intensities;
  }

  void measureFeedback_callBack(const robo7_msgs::MeasureFeedback::ConstPtr &msg)
  {
    if(measure_feedback.id_number != msg->id_number)
    {
      measure_feedback.id_number = msg->id_number;
      measure_feedback.error = msg->error;
      measure_feedback.P_matrix = msg->P_matrix;
      new_measure_received = true;
    }
  }

  void updatePosition()
  {
    //We check if we received a new measure computations
    if(new_measure_received && use_measure)
    {
      // ROS_INFO("Measure feedback");
      //Extract the usefull datas out of the measure received
      position_error(0) = measure_feedback.error.x;
      position_error(1) = measure_feedback.error.y;
      position_error(2) = measure_feedback.error.z;
      for(int i=0; i<3; i++)
      {
        the_P_matrix(0,i) = measure_feedback.P_matrix.line0[i];
        the_P_matrix(1,i) = measure_feedback.P_matrix.line1[i];
        the_P_matrix(2,i) = measure_feedback.P_matrix.line2[i];
      }

      //Update the position based on the error received
      update_position_error();

      //Set the new_measure_value back to zero
      reinitialize_A_W_matrices();
      new_measure_received = false;
      previous_measure_received = true;
    }

    //Dead_reckoning part
    timeUpdate();

    //If we didn't get any measures for a while (and the previous one has already been received)
    if((ros::Time::now().toSec() - prev_mes_time > time_threshold)&&(previous_measure_received)&&use_measure)
    {
      // ROS_INFO("Asking for measure");

      //Indix to show that there is a new request coming
      request_id++;

      //Compute the_P_minus_matrix that is going to be used
      compute_P_minus();

      //Prepare the measure request message
      new_measure_request.time = ros::Time::now();
      new_measure_request.id_number = request_id;
      new_measure_request.current_position = current_position;
      new_measure_request.lidar_scan = the_lidar_scan;
      for(int i=0; i<3; i++)
      {
        new_measure_request.P_minus_matrix.line0.push_back(the_P_minus_matrix(0,i));
        new_measure_request.P_minus_matrix.line1.push_back(the_P_minus_matrix(1,i));
        new_measure_request.P_minus_matrix.line2.push_back(the_P_minus_matrix(2,i));
      }

      //Publish the new request
      new_measure_req_pub.publish( new_measure_request );

      //Update the variables
      previous_measure_received = false; //We have to wait for the answer
      prev_mes_time = ros::Time::now().toSec(); //We update the last measure request
    }

    the_robot_position.linear.x = x_pos;
    the_robot_position.linear.y = y_pos;
    the_robot_position.linear.z = 0;
    the_robot_position.angular.x = 0;
    the_robot_position.angular.y = 0;
    the_robot_position.angular.z = z_angle;

    robot_position.publish( the_robot_position );
    // ROS_INFO("Position published");
  }





private:
  //Position update
  float x_pos;
  float y_pos;
  float z_angle;

  //All the physical dimensions of the robot
  float wheel_radius;
  float wheel_distance;
  float tics_per_rev;
  float pi;

  //Adjustment
  float linear_adjustment;
  float angular_adjustment;

  //Other parameters
  float Dt; //ms - time between two consecutive iterations

  //encoders values
  int encoder_L;
  int encoder_R;
  //Counts
  int count_L;
  int count_R;
  //Prev counts
  int prev_count_L;
  int prev_count_R;
  //Guess the values of both wheel's angular speeds with signs
  float om_L;
  float om_R;
  //Compute the linear and angular velocities
  float ang_dis;
  float lin_dis;

  //Defining the messages of both request and answer
  robo7_msgs::MeasureRequest new_measure_request;
  robo7_msgs::MeasureFeedback measure_feedback;

  //Boolean telling if we want to use the measures or not
  bool use_measure;

  //Boolean for sending and receiving measures
  bool new_measure_received;
  bool previous_measure_received;
  int request_id;

  //Declaration of all the matrices
  Eigen::Matrix3f the_A_matrix;
  Eigen::Matrix3f the_P_minus_matrix;
  Eigen::Matrix3f the_P_matrix;
  Eigen::Matrix3f identity;
  Eigen::Matrix2f the_Q_matrix;
  Eigen::MatrixXf the_W_matrix;
  Eigen::Matrix3f matrix_A_it;
  Eigen::MatrixXf matrix_W_it;

  Eigen::Vector3f position_error;
  geometry_msgs::Twist current_position;
  geometry_msgs::Twist the_robot_position;

  //Variance on both angle and distance of dead_reckoning
  float sigma_d, sigma_a;

  //Time feedback for measurement asking
  int prev_mes_time;
  int time_threshold;

  //the scan sensor_msgs
  sensor_msgs::LaserScan the_lidar_scan;




  void update_position_error()
  {
    x_pos += position_error(0);
    y_pos += position_error(1);
    z_angle += position_error(2);
  }

  void timeUpdate()
  {
    //Update the differents count changes
    encoder_L = count_L - prev_count_L;
    encoder_R = count_R - prev_count_R;

    prev_count_L = count_L;
    prev_count_R = count_R;

    //Guess the values of both wheel's angular speeds with signs
    om_L = angular_motor_distance(encoder_L);
    om_R = angular_motor_distance(encoder_R);

    // Compute the linear and angular velocities
    ang_dis = angular_distance_linearised(om_L, -om_R);
    lin_dis = linear_distance_linearised(om_L, -om_R);

    //Compute the linear distances and angles of the robot
    x_pos += (lin_dis * cos(z_angle)) * (1 + linear_adjustment);
    y_pos += (lin_dis * sin(z_angle)) * (1 + linear_adjustment);
    z_angle += ang_dis * (1 + angular_adjustment);
    z_angle = wrapAngle(z_angle);

    //Then we need to update the different matrices
    matrix_A_it(0,2) = -(lin_dis * sin(z_angle)) * (1 + linear_adjustment);
    matrix_A_it(1,2) = (lin_dis * cos(z_angle)) * (1 + linear_adjustment);
    matrix_W_it(0,0) = cos(z_angle);
    matrix_W_it(1,0) = sin(z_angle);

    //Add those iteration matrices to the main ones
    the_A_matrix = the_A_matrix + matrix_A_it;
    the_W_matrix = the_W_matrix + matrix_W_it;
  }

  void reinitialize_A_W_matrices()
  {
    //Reinitialize the A&W matrices
    the_A_matrix = Eigen::Matrix3f::Zero(3,3);
    the_W_matrix = Eigen::MatrixXf::Zero(3,2);

    //Reinitialize A&W iterations matrices
    matrix_A_it = Eigen::Matrix3f::Identity(3,3);
    matrix_W_it = Eigen::MatrixXf::Zero(3,2);
    matrix_W_it(2,1) = 1;
  }

  void compute_P_minus()
  {
    the_P_minus_matrix = the_A_matrix * the_P_matrix * the_A_matrix.transpose() + the_W_matrix * the_Q_matrix * the_W_matrix.transpose();
  }

  //Other useful function
  int sgn(int v)
  {
    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;
  }

  float angular_motor_distance(int encod)
  {
    return ((2*pi*encod)/(tics_per_rev));
  }

  float linear_distance_linearised(float left_wheel_speed, float right_wheel_speed)
  {
    return wheel_radius*(right_wheel_speed + left_wheel_speed)/2;
  }

  float angular_distance_linearised(float left_wheel_speed, float right_wheel_speed)
  {
    return wheel_radius*(right_wheel_speed - left_wheel_speed)/wheel_distance;
  }

  float wrapAngle( double angle )
  {
    float twoPi = 2.0 * pi;
    return angle - twoPi * floor( angle / twoPi );
  }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_filter");

    kalmanFilter kalmanFilter_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Kalman Filter is turning");

    while(kalmanFilter_.n.ok())
    {
        kalmanFilter_.updatePosition();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}