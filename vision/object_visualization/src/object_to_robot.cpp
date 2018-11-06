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
#include <robo7_msgs/cornerList.h>
#include <robo7_srvs/scanCoord.h>
#include <robo7_srvs/RansacWall.h>
#include <robo7_srvs/ICPAlgorithm.h>
#include <robo7_srvs/callServiceTest.h>
#include <robo7_srvs/PathFollowerSrv.h>
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

    //The service definition
    object_to_robot_frame_srv = n.advertiseService("/localization/object_to_robot", &objectToRobot::transform_Sequence, this);
  }

  bool transform_Sequence(robo7_srvs::callServiceTest::Request &req,
         robo7_srvs::callServiceTest::Response &res)
  {
    robot_position = req.robot_position;
    camera_object_position = req.camera_position;

    object_robot_position_vector = rotation_matrix * object_camera_position_vector + translation_vector;

    object_map_frame = rotation_matrix_2 * object_robot_position_vector + translation_vector_2;

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

  Eigen::Vector3f object_camera_position_vector;
  Eigen::Vector3f object_robot_position_vector;
  Eigen::Vector3f object_map_position_vector;

  Eigen::Matrix3f rotation_matrix;
  Eigen::Vector3f translation_vector;


  void forward_transform()
  {
    object_camera_position_vector(0) = camera_object_position.x;
    object_camera_position_vector(1) = camera_object_position.y;
    object_camera_position_vector(2) = camera_object_position.z;
  }

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_to_robot");

	test_server test_server_;

	ros::Rate loop_rate(100);

	ROS_INFO("The object_to_frame's Service is ready");

	ros::spin();

	return 0;
}
