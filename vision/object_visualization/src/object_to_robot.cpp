//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>
#include <robo7_msgs/Matrix3.h>
#include <robo7_msgs/MeasureRequest.h>
#include <robo7_msgs/MeasureFeedback.h>
#include <robo7_msgs/cornerList.h>
#include <robo7_srvs/objectToRobot.h>
#include <Eigen/Geometry>
using Eigen::MatrixXd;


// Control @ 10 Hz
double control_frequency = 100.0;

class objectToRobot
{
public:
  ros::NodeHandle n;
  ros::Subscriber laser_scan;
  ros::Subscriber map_point_sub;
  ros::ServiceServer object_to_robot_frame_srv;


  objectToRobot()
  {
    //The camera angle
    n.param<float>("/object_to_robot/camera_angle", camera_angle, 0);

    rotation_matrix = Eigen::Matrix3f::Zero(3,3);
    rotation_matrix(0,1) = -1;
    rotation_matrix(1,2) = -1;
    rotation_matrix(2,0) = 1;

    translation_vector(0) = 0.116;
    translation_vector(1) = 0;
    translation_vector(2) = 0.131;

    //The service definition
    object_to_robot_frame_srv = n.advertiseService("/localization/object_to_robot", &objectToRobot::transform_Sequence, this);
  }

  bool transform_Sequence(robo7_srvs::objectToRobot::Request &req,
         robo7_srvs::objectToRobot::Response &res)
  {
    robot_position = req.robot_position;
    camera_object_position = req.camera_position;

    forward_transform();

    object_robot_position_vector = rotation_matrix * object_camera_position_vector + translation_vector;

    object_map_position_vector = rotation_matrix2 * object_robot_position_vector + translation_vector2;

    res.object_in_robot_frame = object_robot_frame;
    res.object_in_map_frame = object_map_frame;
    res.success = true;

    return true;
  }


private:
  //Inputs
  geometry_msgs::Twist robot_position;
  geometry_msgs::Point camera_object_position;

  //Outputs
  geometry_msgs::Point object_robot_frame;
  geometry_msgs::Point object_map_frame;

  //The angle of the camera
  float camera_angle;
  float robot_angle;

  Eigen::Vector3f object_camera_position_vector;
  Eigen::Vector3f object_robot_position_vector;
  Eigen::Vector3f object_map_position_vector;

  //From camera frame to robot frame
  Eigen::Matrix3f rotation_matrix;
  Eigen::Vector3f translation_vector;
  //From robot frame to map frame
  Eigen::Matrix3f rotation_matrix2;
  Eigen::Vector3f translation_vector2;


  void forward_transform()
  {
    object_camera_position_vector(0) = camera_object_position.x;
    object_camera_position_vector(1) = camera_object_position.y;
    object_camera_position_vector(2) = camera_object_position.z;

    rotation_matrix2 = Eigen::Matrix3f::Zero(3,3);
    robot_angle = robot_position.angular.z;
    rotation_matrix2(0,0) = cos(robot_angle);
    rotation_matrix2(0,1) = -sin(robot_angle);
    rotation_matrix2(1,0) = sin(robot_angle);
    rotation_matrix2(1,1) = cos(robot_angle);
    rotation_matrix2(2,2) = 1;

    translation_vector2(0) = robot_position.linear.x;
    translation_vector2(1) = robot_position.linear.y;
    translation_vector2(2) = robot_position.linear.z;
  }

  void back_transform()
  {
    object_robot_frame.x = object_robot_position_vector(0);
    object_robot_frame.y = object_robot_position_vector(1);
    object_robot_frame.z = object_robot_position_vector(2);

    object_map_frame.x = object_map_position_vector(0);
    object_map_frame.y = object_map_position_vector(1);
    object_map_frame.z = object_map_position_vector(2);
  }

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_to_robot");

	objectToRobot objectToRobot_;

	ros::Rate loop_rate(100);

	ROS_INFO("The object_to_frame's Service is ready");

	ros::spin();

	return 0;
}
