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

#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/ICPAlgorithm.h>


// Control @ 10 Hz
double control_frequency = 10.0;

class kalmanFilter
{
public:
  ros::NodeHandle n;
  ros::Subscriber encoder_Left;
  ros::Subscriber encoder_Right;
  ros::Subscriber measure_sub;
  ros::Subscriber scan_sub;
  ros::Subscriber map_point_sub;
  ros::Publisher robot_position;
  ros::Publisher new_measure_req_pub;
  ros::Publisher angle_pub;

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
    n.param<float>("/kalman_filter/sigma_x", sigma_x, 0.05);
    n.param<float>("/kalman_filter/sigma_y", sigma_y, 0.05);
    n.param<float>("/kalman_filter/sigma_angle", sigma_a, 0.1);

    n.param<float>("/kalman_filter/sigma_x_lidar", sigma_x_lidar, 0.05);
    n.param<float>("/kalman_filter/sigma_y_lidar", sigma_y_lidar, 0.05);
    n.param<float>("/kalman_filter/sigma_angle_lidar", sigma_a_lidar, 0.1);

    //Two different methods to test
    n.param<int>("/kalman_filter/which_method", test_method, 0);
    n.param<bool>("/kalman_filter/use_measure", use_measure, false);
    n.param<bool>("/kalman_filter/trust_lidar_and_dead_reckoning", true_kalman, false);

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
    init_time = prev_mes_time;

    //Measure Request/Response initialisations
    request_id = 0;
    previous_measure_received = true;
    new_measure_received = false;

    //Initialize the different matrices
    initialize_matrices();

    encoder_Left = n.subscribe("/l_motor/encoder", 1, &kalmanFilter::encoder_L_callBack, this);
    encoder_Right = n.subscribe("/r_motor/encoder", 1, &kalmanFilter::encoder_R_callBack, this);
    measure_sub = n.subscribe("/localization/meas_update/measure_response", 1, &kalmanFilter::measureFeedback_callBack, this);
    scan_sub = n.subscribe("/scan", 1, &kalmanFilter::scan_callBack, this);
    map_point_sub = n.subscribe("/ras_maze/maze_map/walls_coord_for_icp", 1, &kalmanFilter::maze_map_callBack, this);

    scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
    icp_srv = n.serviceClient<robo7_srvs::ICPAlgorithm>("/localization/icp");

    // robot_position = n.advertise<geometry_msgs::Twist>("/localization/kalman_filter/pos", 10);
    robot_position = n.advertise<geometry_msgs::Twist>("/localization/kalman_filter/position", 1);
    new_measure_req_pub = n.advertise<robo7_msgs::MeasureRequest>("/localization/kalman_filter/measure_request", 1);
    angle_pub = n.advertise<std_msgs::Float32>("/localization/kalman_filter/angle", 1);

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
      the_lidar_scan = *msg;
  }

  void measureFeedback_callBack(const robo7_msgs::MeasureFeedback::ConstPtr &msg)
  {
    if(measure_feedback.id_number != msg->id_number)
    {
      measure_feedback = *msg;
      new_measure_received = true;
    }
  }

  void maze_map_callBack(const robo7_msgs::cornerList::ConstPtr &msg)
  {
    all_wall_points = *msg;
  }

  void updatePosition()
  {

    //Dead_reckoning part
    timeUpdate();

    compute_P_minus();

    update_the_robot_position();

    //If we didn't get any measures for a while (and the previous one has already been received)
    if(use_measure&&(ros::Time::now().toSec() - init_time > 5))
    {
      compute_matrices();
      position_measure_request();
      reinitialize_A_W_matrices();
    }

    robot_position.publish( the_robot_position );
    std_msgs::Float32 angle;
    angle.data = z_angle;
    angle_pub.publish( angle );
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
  bool true_kalman;

  //Boolean for sending and receiving measures
  bool new_measure_received;
  bool previous_measure_received;
  int request_id;

  //Declaration of all the matrices
  Eigen::Matrix3f the_A_matrix;
  Eigen::Matrix3f the_P_minus_matrix;
  Eigen::Matrix3f the_P_matrix;
  Eigen::Matrix3f identity;
  Eigen::Matrix3f the_Q_matrix;
  Eigen::Matrix3f the_W_matrix;
  Eigen::Matrix3f the_H_matrix;
  Eigen::Matrix3f the_R_matrix;
  Eigen::Matrix3f the_V_matrix;
  Eigen::Matrix3f the_K_matrix;

  Eigen::Vector3f position_error;
  geometry_msgs::Twist current_position;
  geometry_msgs::Twist the_robot_position;
  geometry_msgs::Twist corresp_position;

  //Declaration of all the vectors
  Eigen::Vector3f position_error_vect;
  Eigen::Vector3f the_lidar_pos_vect;
  Eigen::Vector3f current_position_vect;

  //Variance on both angle and distance of dead_reckoning
  float sigma_x, sigma_y, sigma_a;
  float sigma_x_lidar, sigma_y_lidar, sigma_a_lidar;


  //Compute the total distance and angle that changed over dead_reckoning
  float tot_dist, tot_angle;

  //Time feedback for measurement asking
  double prev_mes_time;
  double init_time;
  double time_threshold;
  ros::Time time_start;

  //the scan sensor_msgs
  sensor_msgs::LaserScan the_lidar_scan;

  //Two method 0 or 1
  int test_method;

  //Save the position so that you send the corresponding position with the lidar scan
  robo7_msgs::former_position previous_pos;
  robo7_msgs::former_position corresp_pos;
  int corres_pos_index;
  std::vector<robo7_msgs::former_position> saved_position;
  std::vector<robo7_msgs::former_position> positions_for_matrices;
  int number_of_instance_saved;
  int index_for_position;

  //Service request
  robo7_msgs::cornerList all_wall_points;
  geometry_msgs::Twist corrected_position;


  void position_measure_request()
  {
    //Indix to show that there is a new request coming
    request_id++;

    //Prepare the measure request message
    time_start = ros::Time::now();

    robo7_srvs::scanCoord::Request req1;
    robo7_srvs::scanCoord::Response res1;
    req1.robot_position = the_robot_position;
    req1.lidar_scan = the_lidar_scan;
    scan_to_coord_srv.call(req1, res1);

    robo7_srvs::ICPAlgorithm::Request req3;
    robo7_srvs::ICPAlgorithm::Response res3;
    req3.current_position = the_robot_position;
    req3.the_lidar_corners = res1.the_lidar_point_cloud;
    req3.the_wall_corners = all_wall_points;
    icp_srv.call(req3, res3);

    corrected_position = res3.new_position;

    change_pose_to_vector();
    compute_K_matrix();
    compute_the_error();
    update_position_error();
    compute_P_matrix();

  }

  void change_pose_to_vector()
  {
    //current position vector
    current_position_vect(0) = the_robot_position.linear.x;
    current_position_vect(1) = the_robot_position.linear.y;
    current_position_vect(2) = the_robot_position.angular.z;

    //current position vector
    the_lidar_pos_vect(0) = corrected_position.linear.x;
    the_lidar_pos_vect(1) = corrected_position.linear.y;
    the_lidar_pos_vect(2) = corrected_position.angular.z;
  }

  void compute_K_matrix()
  {
    // ROS_INFO("In measure ");
    // ROS_INFO("matrix P minus : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_P_minus_matrix(0,0), the_P_minus_matrix(0,1), the_P_minus_matrix(0,2), the_P_minus_matrix(1,0), the_P_minus_matrix(1,1), the_P_minus_matrix(1,2), the_P_minus_matrix(2,0), the_P_minus_matrix(2,1), the_P_minus_matrix(2,2));
    // ROS_INFO("matrix H : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_H_matrix(0,0), the_H_matrix(0,1), the_H_matrix(0,2), the_H_matrix(1,0), the_H_matrix(1,1), the_H_matrix(1,2), the_H_matrix(2,0), the_H_matrix(2,1), the_H_matrix(2,2));
    // ROS_INFO("matrix H : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_H_matrix(0,0), the_H_matrix(0,1), the_H_matrix(0,2), the_H_matrix(1,0), the_H_matrix(1,1), the_H_matrix(1,2), the_H_matrix(2,0), the_H_matrix(2,1), the_H_matrix(2,2));
    the_K_matrix = the_P_minus_matrix * the_H_matrix * ( the_H_matrix * the_P_minus_matrix * the_H_matrix.transpose() + the_V_matrix * the_R_matrix * the_V_matrix.transpose()).inverse();
    // ROS_INFO("matrx K : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_K_matrix(0,0), the_K_matrix(0,1), the_K_matrix(0,2), the_K_matrix(1,0), the_K_matrix(1,1), the_K_matrix(1,2), the_K_matrix(2,0), the_K_matrix(2,1), the_K_matrix(2,2));
  }

  void compute_the_error()
  {
    // ROS_INFO("Curent position (x,y,thet) : (%lf, %lf, %lf)", current_position_vect(0), current_position_vect(1), current_position_vect(2));
    // ROS_INFO("Lidar Corrected position (x,y,thet) : (%lf, %lf, %lf)", the_lidar_pos_vect(0), the_lidar_pos_vect(1), the_lidar_pos_vect(2));
    position_error_vect = the_K_matrix * ( the_lidar_pos_vect - current_position_vect );
    // ROS_INFO("Position error (x,y,thet) : (%lf, %lf, %lf)", position_error_vect(0), position_error_vect(1), position_error_vect(2));
  }

  void compute_P_matrix()
  {
    // ROS_INFO("matrx K : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_K_matrix(0,0), the_K_matrix(0,1), the_K_matrix(0,2), the_K_matrix(1,0), the_K_matrix(1,1), the_K_matrix(1,2), the_K_matrix(2,0), the_K_matrix(2,1), the_K_matrix(2,2));
    // ROS_INFO("matrix P minus : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_P_minus_matrix(0,0), the_P_minus_matrix(0,1), the_P_minus_matrix(0,2), the_P_minus_matrix(1,0), the_P_minus_matrix(1,1), the_P_minus_matrix(1,2), the_P_minus_matrix(2,0), the_P_minus_matrix(2,1), the_P_minus_matrix(2,2));
    // ROS_INFO("matrix H : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_H_matrix(0,0), the_H_matrix(0,1), the_H_matrix(0,2), the_H_matrix(1,0), the_H_matrix(1,1), the_H_matrix(1,2), the_H_matrix(2,0), the_H_matrix(2,1), the_H_matrix(2,2));
    // inter = (identity - the_K_matrix * the_H_matrix);
    // ROS_INFO("Inter : %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", inter(0,0), inter(0,1), inter(0,2), inter(1,0), inter(1,1), inter(1,2), inter(2,0), inter(2,1), inter(2,2));
    the_P_matrix = (identity - the_K_matrix * the_H_matrix) * the_P_minus_matrix;
    // ROS_INFO("matrix P : %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", the_P_matrix(0,0), the_P_matrix(0,1), the_P_matrix(0,2), the_P_matrix(1,0), the_P_matrix(1,1), the_P_matrix(1,2), the_P_matrix(2,0), the_P_matrix(2,1), the_P_matrix(2,2));
  }


  void update_position_error()
  {
    // ROS_INFO("the error : %lf, %lf, %lf", position_error_vect(0), position_error_vect(1), position_error_vect(2));
    x_pos += position_error_vect(0);
    y_pos += position_error_vect(1);
    z_angle += position_error_vect(2);
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

    //Update of the total distances that the robot moved
    tot_dist += lin_dis;
    tot_angle += ang_dis;

    //Compute the linear distances and angles of the robot
    x_pos += (lin_dis * cos(z_angle)) * (1 + linear_adjustment);
    y_pos += (lin_dis * sin(z_angle)) * (1 + linear_adjustment);
    z_angle += ang_dis * (1 + angular_adjustment);
    z_angle = wrapAngle(z_angle);
  }

  void initialize_matrices()
  {
    reinitialize_A_W_matrices();

    the_P_matrix = Eigen::Matrix3f::Zero(3,3);

    the_Q_matrix = Eigen::Matrix3f::Zero(3,3);

    the_Q_matrix(0,0) = sigma_x;
    the_Q_matrix(1,1) = sigma_y;
    the_Q_matrix(2,2) = sigma_a;

    //Creation of the matrices
    identity = Eigen::Matrix3f::Identity(3,3);
    the_H_matrix = Eigen::Matrix3f::Identity(3,3);
    the_R_matrix = Eigen::Matrix3f::Identity(3,3);
    the_R_matrix(0,0) = sigma_x_lidar;
    the_R_matrix(1,1) = sigma_y_lidar;
    the_R_matrix(2,2) = sigma_a_lidar;
    the_V_matrix = Eigen::Matrix3f::Identity(3,3);
  }

  void reinitialize_A_W_matrices()
  {
    //Reinitialize the A&W matrices
    the_A_matrix = Eigen::Matrix3f::Identity(3,3);
    the_W_matrix = Eigen::MatrixXf::Identity(3,3);
  }

  void compute_P_minus()
  {
    // ROS_INFO("matrx A : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_A_matrix(0,0), the_A_matrix(0,1), the_A_matrix(0,2), the_A_matrix(1,0), the_A_matrix(1,1), the_A_matrix(1,2), the_A_matrix(2,0), the_A_matrix(2,1), the_A_matrix(2,2));
    // ROS_INFO("matrix P: %f, %f, %f, %f, %f, %f, %f, %f, %f", the_P_matrix(0,0), the_P_matrix(0,1), the_P_matrix(0,2), the_P_matrix(1,0), the_P_matrix(1,1), the_P_matrix(1,2), the_P_matrix(2,0), the_P_matrix(2,1), the_P_matrix(2,2));
    // ROS_INFO("matrix W : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_W_matrix(0,0), the_W_matrix(0,1), the_W_matrix(0,2), the_W_matrix(1,0), the_W_matrix(1,1), the_W_matrix(1,2), the_W_matrix(2,0), the_W_matrix(2,1), the_W_matrix(2,2));
    // ROS_INFO("matrix Q : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_Q_matrix(0,0), the_Q_matrix(0,1), the_Q_matrix(0,2), the_Q_matrix(1,0), the_Q_matrix(1,1), the_Q_matrix(1,2), the_Q_matrix(2,0), the_Q_matrix(2,1), the_Q_matrix(2,2));
    // inter = (identity - the_K_matrix * the_H_matrix);
    // ROS_INFO("Inter : %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", inter(0,0), inter(0,1), inter(0,2), inter(1,0), inter(1,1), inter(1,2), inter(2,0), inter(2,1), inter(2,2));
    the_P_minus_matrix = the_A_matrix * the_P_matrix * the_A_matrix.transpose() + the_W_matrix * the_Q_matrix * the_W_matrix.transpose();
  }

  void update_the_robot_position()
  {
    time_now = ros::Time::now();
    the_robot_position.linear.x = x_pos;
    the_robot_position.linear.y = y_pos;
    the_robot_position.linear.z = 0;
    the_robot_position.angular.x = 0;
    the_robot_position.angular.y = 0;
    the_robot_position.angular.z = z_angle;
  }

  void compute_matrices()
  {
    if(test_method >= 0)
    {
      the_A_matrix(0,2) = -(lin_dis * sin(z_angle)) * (1 + linear_adjustment);
      the_A_matrix(1,2) = (lin_dis * cos(z_angle)) * (1 + linear_adjustment);
      the_W_matrix(0,0) = cos(z_angle);
      the_W_matrix(1,1) = sin(z_angle);
    }

    //Compute the_P_minus_matrix that is going to be used
    compute_P_minus();
  }

  void position_for_lidar()
  {
    diff_time = time_now.toSec() - the_lidar_scan.header.stamp.toSec();
    corresp_pos = the_robot_position;
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
