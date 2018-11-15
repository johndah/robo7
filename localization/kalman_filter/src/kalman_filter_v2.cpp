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
#include <robo7_msgs/former_position.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_msgs/robotPositionTest.h>
#include <robo7_msgs/the_robot_position.h>

#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/ICPAlgorithm.h>


// Control @ 10 Hz
double control_frequency = 100.0;

class kalmanFilter
{
public:
  ros::NodeHandle n;
  //Subscribers
  ros::Subscriber encoder_Left;
  ros::Subscriber encoder_Right;
  ros::Subscriber measure_sub;
  ros::Subscriber scan_sub;
  ros::Subscriber map_point_sub;
  //Publishers
  ros::Publisher robot_position;
  ros::Publisher robot_position_pub;
  ros::Publisher test_pub;
  //Services
  ros::ServiceClient scan_to_coord_srv;
  ros::ServiceClient icp_srv;

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

    //The errors values
    n.param<float>("/kalman_filter/sigma_distance", sigma_d, 0.05);
    n.param<float>("/kalman_filter/sigma_angle", sigma_a, 0.1);
    n.param<float>("/kalman_filter/sigma_distance_lidar", sigma_d_lidar, 0.05);
    n.param<float>("/kalman_filter/sigma_angle_lidar", sigma_a_lidar, 0.1);

    ROS_INFO("Sigmas : %f, %f ,%f, %f", sigma_d, sigma_a, sigma_d_lidar, sigma_a_lidar);
    //Decide if we only go for dead_reckoning or EKF
    n.param<bool>("/kalman_filter/use_measure", use_measure, false);
    n.param<bool>("/kalman_filter/use_dead_reckoning", use_dead_reckoning, false);

    //Initialize the different matrices
    initialize_variables();
    encoder_saver_initialization();
    time_start = ros::Time::now();

    encoder_Left = n.subscribe("/l_motor/encoder", 1, &kalmanFilter::encoder_L_callBack, this);
    encoder_Right = n.subscribe("/r_motor/encoder", 1, &kalmanFilter::encoder_R_callBack, this);
    scan_sub = n.subscribe("/scan", 1, &kalmanFilter::scan_callBack, this);
    map_point_sub = n.subscribe("/ras_maze/maze_map/walls_coord_for_icp", 1, &kalmanFilter::maze_map_callBack, this);

    scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
    icp_srv = n.serviceClient<robo7_srvs::ICPAlgorithm>("/localization/icp");

    robot_position_pub = n.advertise<robo7_msgs::the_robot_position>("/localization/kalman_filter/position_timed", 1);
    robot_position = n.advertise<geometry_msgs::Twist>("/localization/kalman_filter/position", 1);

    ROS_INFO("EKF initialisation done");
  }

  void encoder_L_callBack(const phidgets::motor_encoder::ConstPtr &msg)
  {
    if((msg->header.seq != left_encoder_msg.header.seq)&&(msg->header.seq >= 1))
    {
      left_encoder_msg = *msg;
      update_left_encoder_saver();
    }
  }

  void encoder_R_callBack(const phidgets::motor_encoder::ConstPtr &msg)
  {
    if(msg->header.seq != right_encoder_msg.header.seq)
    {
      right_encoder_msg = *msg;
      update_right_encoder_saver();
    }
  }

  void scan_callBack(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    if(msg->header.seq != the_lidar_scan.header.seq)
    {
      the_lidar_scan = *msg;
      new_lidar_scan = true;
    }
  }

  void maze_map_callBack(const robo7_msgs::cornerList::ConstPtr &msg)
  {
    all_wall_points = *msg;
  }

  void updatePosition()
  {
    robo7_msgs::robotPositionTest test;

    if(use_dead_reckoning)
    {
      dead_reckoning_position();
    }

    if((new_lidar_scan)&&(ros::Time::now().toSec() - time_start.toSec() > 5))
    {
      //First, find the corresponding times for the lidar scan and encoders
      find_the_corresponding_times();

      //Do the time update with the encoders found -> dead_reckoning
      if(use_dead_reckoning)
      {
        time_Update();
      }

      //Then we need to do the Measurement Update
      if(use_measure)
      {
        measurement_update();
      }

      if(!use_dead_reckoning)
      {
        the_robot_position.position = lidar_position;
      }

      //Update the header of the newly computed robot_position
      the_robot_position.header.seq++;
      the_robot_position.header.stamp = the_lidar_scan.header.stamp;
      the_robot_position.the_lidar_scan = the_lidar_scan;

      //Update the robot position
      estimated_robot_position = the_robot_position;

      //Wait for a new lidar scan before the next update
      new_lidar_scan = false;
    }

    robot_position_pub.publish( estimated_robot_position );
    robot_position.publish( the_robot_position.position );
  }


private:
  //The main functions
  void encoder_saver_initialization()
  {
    //Define the size of the saver
    number_of_instance_saved = (int)control_frequency;

    //Create the two savers
    right_encoder_saver = std::vector<phidgets::motor_encoder>(number_of_instance_saved, right_encoder_msg);
    left_encoder_saver = std::vector<phidgets::motor_encoder>(number_of_instance_saved, left_encoder_msg);
  }

  void update_right_encoder_saver()
  {
    right_encoder_saver.erase(right_encoder_saver.begin());
    right_encoder_saver.push_back(right_encoder_msg);
  }

  void update_left_encoder_saver()
  {
    left_encoder_saver.erase(left_encoder_saver.begin());
    left_encoder_saver.push_back(left_encoder_msg);
  }

  void print_encoder_times()
  {
    for(int i=0; i < number_of_instance_saved; i++)
    {
      ROS_INFO("Left : %d, %lf", left_encoder_saver[i].header.seq, left_encoder_saver[i].header.stamp.toSec());
      ROS_INFO("Right : %d, %lf", right_encoder_saver[i].header.seq, right_encoder_saver[i].header.stamp.toSec());
    }
  }

  void find_the_corresponding_times()
  {
    //Initialization
    left_encoder_corresp = left_encoder_saver[left_encoder_saver.size()-1];
    right_encoder_corresp = right_encoder_saver[right_encoder_saver.size()-1];
    left_encoder_corresp_index = static_cast<int>(left_encoder_saver.size()) - 1;
    right_encoder_corresp_index = static_cast<int>(right_encoder_saver.size()) - 1;

    for(int i = static_cast<int>(left_encoder_saver.size()) - 2; i > -1; i--)
    {
      if(std::abs(left_encoder_corresp.header.stamp.toSec() - the_lidar_scan.header.stamp.toSec())
            > std::abs(left_encoder_saver[i].header.stamp.toSec() - the_lidar_scan.header.stamp.toSec()))
      {
        left_encoder_corresp = left_encoder_saver[i];
        left_encoder_corresp_index = i;
      }
      if(std::abs(right_encoder_corresp.header.stamp.toSec() - the_lidar_scan.header.stamp.toSec())
            > std::abs(right_encoder_saver[i].header.stamp.toSec() - the_lidar_scan.header.stamp.toSec()))
      {
        right_encoder_corresp = right_encoder_saver[i];
        right_encoder_corresp_index = i;
      }
    }
  }

  void time_Update()
  {
    //Save the angle of the previous robot_position (before time update)
    //because it is needed for A and W matrices
    previous_angle = the_robot_position.position.angular.z;

    //First use the dead_reckoning motion to project the state ahead
    dead_reckoning_update();

    //Update both A and W matrices
    update_A_W_matrices();

    //Then project the error covariance matrice ahead
    project_error_covariance();
  }

  void measurement_update()
  {
    //First we compute the Kalman gain
    compute_K_matrix();

    //Then we need to update the estimate with the measurement z
    update_estimate();

    //Finally, we end up with computing the new P matrix
    compute_P_matrix();
  }

  void dead_reckoning_update()
  {
    //Use the extracted encoders value to update the counts
    count_L = left_encoder_corresp.count;
    count_R = right_encoder_corresp.count;

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
    the_robot_position.position.linear.x += (lin_dis * cos(the_robot_position.position.angular.z)) * (1 + linear_adjustment);
    the_robot_position.position.linear.y += (lin_dis * sin(the_robot_position.position.angular.z)) * (1 + linear_adjustment);
    the_robot_position.position.angular.z += ang_dis * (1 + angular_adjustment);
    the_robot_position.position.angular.z = wrapAngle(the_robot_position.position.angular.z);
  }

  void update_A_W_matrices()
  {
    //Update the A matrice
    the_A_matrix = Eigen::Matrix3f::Identity(3,3);
    the_A_matrix(0,2) = -lin_dis * sin(previous_angle);
    the_A_matrix(1,2) = lin_dis * cos(previous_angle);

    //Update the W matrice
    the_W_matrix(0,0) = cos(previous_angle);
    the_W_matrix(1,0) = sin(previous_angle);
    the_W_matrix(2,1) = 1;
  }

  void project_error_covariance()
  {
    // ROS_INFO("matrx A : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_A_matrix(0,0), the_A_matrix(0,1), the_A_matrix(0,2), the_A_matrix(1,0), the_A_matrix(1,1), the_A_matrix(1,2), the_A_matrix(2,0), the_A_matrix(2,1), the_A_matrix(2,2));
    // ROS_INFO("matrix P: %f, %f, %f, %f, %f, %f, %f, %f, %f", the_P_matrix(0,0), the_P_matrix(0,1), the_P_matrix(0,2), the_P_matrix(1,0), the_P_matrix(1,1), the_P_matrix(1,2), the_P_matrix(2,0), the_P_matrix(2,1), the_P_matrix(2,2));
    // ROS_INFO("matrix W : %f, %f, %f, %f, %f, %f", the_W_matrix(0,0), the_W_matrix(0,1), the_W_matrix(1,0), the_W_matrix(1,1), the_W_matrix(2,0), the_W_matrix(2,1));
    // ROS_INFO("matrix Q : %f, %f, %f, %f", the_Q_matrix(0,0), the_Q_matrix(0,1), the_Q_matrix(1,0), the_Q_matrix(1,1));
    the_P_minus_matrix = the_A_matrix * the_P_matrix * the_A_matrix.transpose() + the_W_matrix * the_Q_matrix * the_W_matrix.transpose();
  }

  void compute_K_matrix()
  {
    // ROS_INFO("In measure ");
    // ROS_INFO("matrix P minus : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_P_minus_matrix(0,0), the_P_minus_matrix(0,1), the_P_minus_matrix(0,2), the_P_minus_matrix(1,0), the_P_minus_matrix(1,1), the_P_minus_matrix(1,2), the_P_minus_matrix(2,0), the_P_minus_matrix(2,1), the_P_minus_matrix(2,2));
    // ROS_INFO("matrix H : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_H_matrix(0,0), the_H_matrix(0,1), the_H_matrix(0,2), the_H_matrix(1,0), the_H_matrix(1,1), the_H_matrix(1,2), the_H_matrix(2,0), the_H_matrix(2,1), the_H_matrix(2,2));
    // ROS_INFO("matrix V : %f, %f, %f, %f, %f, %f", the_V_matrix(0,0), the_V_matrix(0,1), the_V_matrix(1,0), the_V_matrix(1,1), the_V_matrix(2,0), the_V_matrix(2,1));
    // ROS_INFO("matrix R : %f, %f, %f, %f", the_R_matrix(0,0), the_R_matrix(0,1), the_R_matrix(1,0), the_R_matrix(1,1));
    the_K_matrix = the_P_minus_matrix * the_H_matrix.transpose() * ( the_H_matrix * the_P_minus_matrix * the_H_matrix.transpose() + the_V_matrix * the_R_matrix * the_V_matrix.transpose()).inverse();
    // ROS_INFO("matrix K : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_K_matrix(0,0), the_K_matrix(0,1), the_K_matrix(0,2), the_K_matrix(1,0), the_K_matrix(1,1), the_K_matrix(1,2), the_K_matrix(2,0), the_K_matrix(2,1), the_K_matrix(2,2));
  }

  void update_estimate()
  {
    //First we found the corresponding z measure with the service made for it
    robo7_srvs::scanCoord::Request req1;
    robo7_srvs::scanCoord::Response res1;
    req1.robot_position = the_robot_position.position;
    req1.lidar_scan = the_lidar_scan;
    scan_to_coord_srv.call(req1, res1);

    robo7_srvs::ICPAlgorithm::Request req2;
    robo7_srvs::ICPAlgorithm::Response res2;
    req2.current_position = the_robot_position.position;
    req2.the_lidar_corners = res1.the_lidar_point_cloud;
    req2.the_wall_corners = all_wall_points;
    icp_srv.call(req2, res2);

    //Extract the corrected position end change it into a usable vector for EKF
    lidar_position = res2.new_position;

    //Then we need to change those position messages into vectors for the computation
    change_pose_to_vector();

    //Finally, we compute the upadte estimate
    compute_the_estimate();

    //Then change it back into the normal position msg
    change_vector_to_pose();
  }

  void change_pose_to_vector()
  {
    //current position vector
    the_x_minus_vector(0) = the_robot_position.position.linear.x;
    the_x_minus_vector(1) = the_robot_position.position.linear.y;
    the_x_minus_vector(2) = the_robot_position.position.angular.z;

    //current position vector
    the_z_vector(0) = lidar_position.linear.x;
    the_z_vector(1) = lidar_position.linear.y;
    the_z_vector(2) = lidar_position.angular.z;
  }

  void compute_the_estimate()
  {
    // ROS_INFO("Curent position (x,y,thet) : (%lf, %lf, %lf)", the_x_minus_vector(0), the_x_minus_vector(1), the_x_minus_vector(2));
    // ROS_INFO("Lidar Corrected position (x,y,thet) : (%lf, %lf, %lf)", the_z_vector(0), the_z_vector(1), the_z_vector(2));
    the_x_vector = the_x_minus_vector +  ( the_z_vector - the_x_minus_vector );
    // ROS_INFO("Position error (x,y,thet) : (%lf, %lf, %lf)", the_x_vector(0), the_x_vector(1), the_x_vector(2));
  }

  void change_vector_to_pose()
  {
    //current position vector
    the_robot_position.position.linear.x = the_x_vector(0);
    the_robot_position.position.linear.y = the_x_vector(1);
    the_robot_position.position.angular.z = the_x_vector(2);
  }

  void compute_P_matrix()
  {
    // ROS_INFO("matrx K : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_K_matrix(0,0), the_K_matrix(0,1), the_K_matrix(0,2), the_K_matrix(1,0), the_K_matrix(1,1), the_K_matrix(1,2), the_K_matrix(2,0), the_K_matrix(2,1), the_K_matrix(2,2));
    // ROS_INFO("matrix P minus : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_P_minus_matrix(0,0), the_P_minus_matrix(0,1), the_P_minus_matrix(0,2), the_P_minus_matrix(1,0), the_P_minus_matrix(1,1), the_P_minus_matrix(1,2), the_P_minus_matrix(2,0), the_P_minus_matrix(2,1), the_P_minus_matrix(2,2));
    // ROS_INFO("matrix H : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_H_matrix(0,0), the_H_matrix(0,1), the_H_matrix(0,2), the_H_matrix(1,0), the_H_matrix(1,1), the_H_matrix(1,2), the_H_matrix(2,0), the_H_matrix(2,1), the_H_matrix(2,2));
    inter = (identity - the_K_matrix * the_H_matrix);
    // ROS_INFO("Inter : %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", inter(0,0), inter(0,1), inter(0,2), inter(1,0), inter(1,1), inter(1,2), inter(2,0), inter(2,1), inter(2,2));
    the_P_matrix = (identity - the_K_matrix * the_H_matrix) * the_P_minus_matrix;
    // ROS_INFO("matrix P : %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", the_P_matrix(0,0), the_P_matrix(0,1), the_P_matrix(0,2), the_P_matrix(1,0), the_P_matrix(1,1), the_P_matrix(1,2), the_P_matrix(2,0), the_P_matrix(2,1), the_P_matrix(2,2));
  }

  void initialize_variables()
  {
    //Robots inner parameters
    wheel_radius = 97.6/2000.0; //m
    wheel_distance = 217.3/1000.0; //m
    tics_per_rev = 897.96;
    pi = 3.14159265358979323846;

    //Initialisation of the dead_reckoning algorithm
    encoder_R = 0;
    encoder_L = 0;
    count_L = 0;
    count_R = 0;
    prev_count_L = 0;
    prev_count_R = 0;

    //Initialisation of the dead_reckoning algorithm
    encoder_R_blind = 0;
    encoder_L_blind = 0;
    count_L_blind = 0;
    count_R_blind = 0;
    prev_count_L_blind = 0;
    prev_count_R_blind = 0;

    //Initialize the position of the robot -> time will come afterward
    the_robot_position.position.linear.x = x_pos;
    the_robot_position.position.linear.y = y_pos;
    the_robot_position.position.linear.z = 0;
    the_robot_position.position.angular.x = 0;
    the_robot_position.position.angular.y = 0;
    the_robot_position.position.angular.z = z_angle;

    estimated_robot_position = the_robot_position;

    //Initialize the matrices for EKF
    initialize_matrices();
  }

  void initialize_matrices()
  {
    //The time update matrices
    the_P_minus_matrix = Eigen::Matrix3f::Zero(3,3);
    the_A_matrix = Eigen::Matrix3f::Identity(3,3);
    the_W_matrix = Eigen::MatrixXf::Zero(3,2);
    the_W_matrix(2,1) = 1;
    the_Q_matrix = Eigen::Matrix2f::Zero(2,2);
    the_Q_matrix(0,0) = sigma_d;
    the_Q_matrix(1,1) = sigma_a;

    //The measurements update matrices
    the_P_matrix = Eigen::Matrix3f::Zero(3,3);
    identity = Eigen::Matrix3f::Identity(3,3);
    the_H_matrix = Eigen::Matrix3f::Identity(3,3);
    the_R_matrix = Eigen::Matrix2f::Zero(2,2);
    the_R_matrix(0,0) = sigma_d_lidar;
    the_R_matrix(1,1) = sigma_a_lidar;
    the_V_matrix = Eigen::MatrixXf::Zero(3,2);
    the_V_matrix(0,0) = 1;
    the_V_matrix(1,0) = 1;
    the_V_matrix(2,1) = 1;
  }

  void dead_reckoning_position()
  {
    //Pick up the next encoder values that shows up
    left_encoder_corresp_blind = left_encoder_saver[left_encoder_corresp_index];
    right_encoder_corresp_blind = right_encoder_saver[right_encoder_corresp_index];

    //Use the extracted encoders value to update the counts
    count_L_blind = left_encoder_corresp_blind.count;
    count_R_blind = right_encoder_corresp_blind.count;

    //Update the differents count changes
    encoder_L_blind = count_L_blind - prev_count_L_blind;
    encoder_R_blind = count_R_blind - prev_count_R_blind;

    prev_count_L_blind = count_L_blind;
    prev_count_R_blind = count_R_blind;

    //Guess the values of both wheel's angular speeds with signs
    om_L_blind = angular_motor_distance(encoder_L_blind);
    om_R_blind = angular_motor_distance(encoder_R_blind);

    // Compute the linear and angular velocities
    ang_dis_blind = angular_distance_linearised(om_L_blind, -om_R_blind);
    lin_dis_blind = linear_distance_linearised(om_L_blind, -om_R_blind);

    //Compute the linear distances and angles of the robot
    estimated_robot_position.position.linear.x += (lin_dis_blind * cos(estimated_robot_position.position.angular.z)) * (1 + linear_adjustment);
    estimated_robot_position.position.linear.y += (lin_dis_blind * sin(estimated_robot_position.position.angular.z)) * (1 + linear_adjustment);
    estimated_robot_position.position.angular.z += ang_dis_blind * (1 + angular_adjustment);
    estimated_robot_position.position.angular.z = wrapAngle(estimated_robot_position.position.angular.z);
  }

  //The variables
  //Position update
  float x_pos, y_pos, z_angle, previous_angle;

  //All the physical dimensions of the robot
  float wheel_radius, wheel_distance, tics_per_rev, pi;

  //Adjustment
  float linear_adjustment, angular_adjustment;

  //The output of EKF
  robo7_msgs::the_robot_position the_robot_position, estimated_robot_position;

  //encoders values
  int encoder_L, encoder_R, encoder_L_blind, encoder_R_blind;
  //Counts
  int count_L, count_R, count_L_blind, count_R_blind;
  //Prev counts
  int prev_count_L, prev_count_R, prev_count_L_blind, prev_count_R_blind;
  //Guess the values of both wheel's angular speeds with signs
  float om_L, om_R, om_L_blind, om_R_blind;
  //Compute the linear and angular velocities
  float ang_dis, lin_dis, ang_dis_blind, lin_dis_blind;

  //Boolean telling if we want to use the measures or not
  bool use_measure;
  bool use_dead_reckoning;

  //Boolean for sending and receiving measures
  bool new_lidar_scan;

  //Declaration of all the matrices
  //Time update matrices
  Eigen::Matrix3f the_P_minus_matrix;
  Eigen::Matrix3f the_A_matrix;
  Eigen::Matrix2f the_Q_matrix;
  Eigen::MatrixXf the_W_matrix;
  //The measurements matrices
  Eigen::Matrix3f the_P_matrix;
  Eigen::Matrix3f the_H_matrix;
  Eigen::Matrix2f the_R_matrix;
  Eigen::MatrixXf the_V_matrix;
  Eigen::Matrix3f the_K_matrix;
  Eigen::Matrix3f identity;

  Eigen::Matrix3f inter;

  //Declaration of all the vectors
  Eigen::Vector3f the_x_vector, the_x_minus_vector, the_z_vector;

  //Variance on both angle and distance of dead_reckoning
  float sigma_d, sigma_a;
  float sigma_d_lidar, sigma_a_lidar;

  //the scan sensor_msgs
  sensor_msgs::LaserScan the_lidar_scan;
  geometry_msgs::Twist lidar_position;
  robo7_msgs::cornerList all_wall_points;

  //The savers definition
  phidgets::motor_encoder left_encoder_msg, right_encoder_msg;
  int number_of_instance_saved;
  std::vector<phidgets::motor_encoder> left_encoder_saver, right_encoder_saver;
  phidgets::motor_encoder left_encoder_corresp, right_encoder_corresp;
  int left_encoder_corresp_index, right_encoder_corresp_index;

  //Initial time that leave the robot the time to start everything before the computations
  ros::Time time_start;

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

  float linear_distance_linearised(float left_wheel_distance, float right_wheel_distance)
  {
    return wheel_radius*(right_wheel_distance + left_wheel_distance)/2;
  }

  float angular_distance_linearised(float left_wheel_distance, float right_wheel_distance)
  {
    return wheel_radius*(right_wheel_distance - left_wheel_distance)/wheel_distance;
  }

  float wrapAngle( double angle )
  {
    float twoPi = 2.0 * pi;
    return angle - twoPi * floor( angle / twoPi );
  }

  void print_robot_position()
  {
    ROS_INFO("(x,y,theta) = (%lf, %lf, %lf)", the_robot_position.position.linear.x, the_robot_position.position.linear.y, the_robot_position.position.angular.z);
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
