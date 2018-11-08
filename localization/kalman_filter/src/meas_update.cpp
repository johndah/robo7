//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>
#include <robo7_msgs/Matrix3.h>
#include <robo7_msgs/MeasureRequest.h>
#include <robo7_msgs/MeasureFeedback.h>
#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/RansacWall.h>
#include <robo7_srvs/ICPAlgorithm.h>



// Control @ 10 Hz
double control_frequency = 100.0;

class meas_update
{
public:
  ros::NodeHandle n;
  ros::Subscriber meas_request_sub;
  ros::Subscriber map_point_sub;
  ros::Publisher meas_result_pub;
  ros::ServiceClient scan_to_coord_srv;
  ros::ServiceClient ransac_srv;
  ros::ServiceClient icp_srv;

  meas_update()
  {
    ROS_INFO("Starting Measure update");
    //Measurement erros
    n.param<float>("/meas_update/sigma_x", sigma_x, 0.1);
    n.param<float>("/meas_update/sigma_y", sigma_y, 0.1);
    n.param<float>("/meas_update/sigma_angle", sigma_theta, 0.05);

    //Creation of the matrices
    identity = Eigen::Matrix3f::Identity(3,3);
    the_H_matrix = Eigen::Matrix3f::Identity(3,3);
    the_R_matrix(0,0) = sigma_x;
    the_R_matrix(1,1) = sigma_x;
    the_R_matrix(2,2) = sigma_theta;
    the_V_matrix = Eigen::Matrix3f::Identity(3,3);

    new_request = false;

    meas_request_sub = n.subscribe("/localization/kalman_filter/measure_request", 10, &meas_update::measure_request_callBack, this);
    meas_result_pub = n.advertise<robo7_msgs::MeasureFeedback>("/localization/meas_update/measure_response", 10);

    map_point_sub = n.subscribe("/ras_maze/maze_map/walls_coord_for_icp", 1, &meas_update::maze_map_callBack, this);
    scan_to_coord_srv = n.serviceClient<robo7_srvs::scanCoord>("/localization/scan_service");
    ransac_srv = n.serviceClient<robo7_srvs::RansacWall>("/localization/ransac");
    icp_srv = n.serviceClient<robo7_srvs::ICPAlgorithm>("/localization/icp");
    ROS_INFO("Measure update initialisation done");
  }

  void measure_request_callBack(const robo7_msgs::MeasureRequest::ConstPtr &msg)
  {
    //We check if the current request is not the same as before
    if((new_measure_request.id_number != msg->id_number)&&(msg->id_number >= 1))
    {
      // ROS_INFO("New Measure");
      new_measure_request.time = msg->time;;
      new_measure_request.id_number = msg->id_number;
      new_measure_request.current_position = msg->current_position;
      new_measure_request.lidar_scan = msg->lidar_scan;
      new_measure_request.P_minus_matrix = msg->P_minus_matrix;
      new_request = true;
    }
  }

  void maze_map_callBack(const robo7_msgs::cornerList::ConstPtr &msg)
  {
    all_wall_points = *msg;
  }

  void measurement_update()
  {
    //If a new request arrived, then we start computing otherwise we do nothing
    if(new_request)
    {
      // ROS_INFO("Treating New Measure");
      //Call for the scan_to_coordinate node : it will transform the coordinates of the laser scan
      //into the map frame
      robo7_srvs::scanCoord::Request req1;
      robo7_srvs::scanCoord::Response res1;
      req1.robot_position = new_measure_request.current_position;
      req1.lidar_scan = new_measure_request.lidar_scan;
      scan_to_coord_srv.call(req1, res1);

      robo7_srvs::ICPAlgorithm::Request req3;
      robo7_srvs::ICPAlgorithm::Response res3;
      req3.current_position = new_measure_request.current_position;
      req3.the_lidar_corners = res1.the_lidar_point_cloud;
      req3.the_wall_corners = all_wall_points;
      icp_srv.call(req3, res3);

      //Extract the corrected position end change it into a usable vector for EKF
      current_position = new_measure_request.current_position;
      corrected_position = res3.new_position;
      change_pose_to_vector();

      //Once we have extracted the position out of the lidar measurement, it is time to do
      //the EKF measurement step
      extract_P_minus_matrix();
      compute_K_matrix();
      compute_the_error();
      compute_P_matrix();

      //Then we publish the result message
      measure_feedback.error.x = position_error_vect(0);
      measure_feedback.error.y = position_error_vect(1);
      measure_feedback.error.z = position_error_vect(2);
      measure_feedback.P_matrix.line0.clear();
      measure_feedback.P_matrix.line1.clear();
      measure_feedback.P_matrix.line2.clear();
      measure_feedback.position_corrected = corrected_position;
      for(int i=0; i<3; i++)
      {
        measure_feedback.P_matrix.line0.push_back(the_P_matrix(0,i));
        measure_feedback.P_matrix.line1.push_back(the_P_matrix(1,i));
        measure_feedback.P_matrix.line2.push_back(the_P_matrix(2,i));
      }


      //Once the request is treated, we wait for another one
      new_request = false;

      measure_feedback.id_number++;
      //Publish the message back to the kalman_filter
      meas_result_pub.publish( measure_feedback );
    }
  }


private:
  //All the messages
  robo7_msgs::MeasureRequest new_measure_request;
  robo7_msgs::MeasureFeedback measure_feedback;

  geometry_msgs::Twist corrected_position;
  geometry_msgs::Twist current_position;


  //Declaration of all the matrices
  Eigen::Matrix3f the_H_matrix;
  Eigen::Matrix3f the_K_matrix;
  Eigen::Matrix3f the_P_minus_matrix;
  Eigen::Matrix3f the_P_matrix;
  Eigen::Matrix3f identity;
  Eigen::Matrix3f inter;
  Eigen::Matrix3f the_R_matrix;
  Eigen::Matrix3f the_V_matrix;

  //Declaration of all the vectors
  Eigen::Vector3f position_error_vect;
  Eigen::Vector3f the_lidar_pos_vect;
  Eigen::Vector3f current_position_vect;

  //Error estimation on the lidar measurement
  float sigma_x, sigma_y, sigma_theta;

  //New request received
  bool new_request;

  robo7_msgs::cornerList all_wall_points;


  void extract_P_minus_matrix()
  {
    for(int i=0; i<3; i++)
    {
      the_P_minus_matrix(0,i) = new_measure_request.P_minus_matrix.line0[i];
      the_P_minus_matrix(1,i) = new_measure_request.P_minus_matrix.line1[i];
      the_P_minus_matrix(2,i) = new_measure_request.P_minus_matrix.line2[i];
    }
  }

  void compute_K_matrix()
  {
    // ROS_INFO("In measure ");
    // ROS_INFO("matrix P minus : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_P_minus_matrix(0,0), the_P_minus_matrix(0,1), the_P_minus_matrix(0,2), the_P_minus_matrix(1,0), the_P_minus_matrix(1,1), the_P_minus_matrix(1,2), the_P_minus_matrix(2,0), the_P_minus_matrix(2,1), the_P_minus_matrix(2,2));
    // ROS_INFO("matrix H : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_H_matrix(0,0), the_H_matrix(0,1), the_H_matrix(0,2), the_H_matrix(1,0), the_H_matrix(1,1), the_H_matrix(1,2), the_H_matrix(2,0), the_H_matrix(2,1), the_H_matrix(2,2));
    the_K_matrix = the_P_minus_matrix * the_H_matrix * ( the_H_matrix * the_P_minus_matrix * the_H_matrix.transpose() + the_V_matrix * the_R_matrix * the_V_matrix.transpose()).inverse();
    // ROS_INFO("matrx K : %f, %f, %f, %f, %f, %f, %f, %f, %f", the_K_matrix(0,0), the_K_matrix(0,1), the_K_matrix(0,2), the_K_matrix(1,0), the_K_matrix(1,1), the_K_matrix(1,2), the_K_matrix(2,0), the_K_matrix(2,1), the_K_matrix(2,2));
  }

  void compute_the_error()
  {
    ROS_INFO("Curent position (x,y,thet) : (%lf, %lf, %lf)", current_position_vect(0), current_position_vect(1), current_position_vect(2));
    ROS_INFO("Lidar Corrected position (x,y,thet) : (%lf, %lf, %lf)", the_lidar_pos_vect(0), the_lidar_pos_vect(1), the_lidar_pos_vect(2));
    position_error_vect = the_K_matrix * ( the_lidar_pos_vect - current_position_vect );
    ROS_INFO("Position error (x,y,thet) : (%lf, %lf, %lf)", position_error_vect(0), position_error_vect(1), position_error_vect(2));
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

  void change_pose_to_vector()
  {
    //current position vector
    current_position_vect(0) = current_position.linear.x;
    current_position_vect(1) = current_position.linear.y;
    current_position_vect(2) = current_position.angular.z;

    //current position vector
    the_lidar_pos_vect(0) = corrected_position.linear.x;
    the_lidar_pos_vect(1) = corrected_position.linear.y;
    the_lidar_pos_vect(2) = corrected_position.angular.z;
  }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "meas_update");

    meas_update meas_update_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Measurement_Update is turning");

    while(meas_update_.n.ok())
    {
        meas_update_.measurement_update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
