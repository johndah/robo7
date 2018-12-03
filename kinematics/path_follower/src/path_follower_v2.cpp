// #include <algorithm>
#include <stdlib.h>     /* abs */
#include <ros/ros.h>
//Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <robo7_msgs/the_robot_position.h>
#include <robo7_msgs/target_trajectory.h>
#include <robo7_msgs/wallPoint.h>
#include <robo7_msgs/trajectory.h>
#include <robo7_msgs/detectedState.h>
//Services
#include <robo7_srvs/PathFollower2.h>
#include <robo7_srvs/PureRotation.h>
#include <robo7_srvs/IsGridOccupied.h>
#include <robo7_srvs/MoveStraight.h>
#include <robo7_srvs/FilterOn.h>

float control_frequency = 10.0;
float pi = 3.14;

class path_follower_v2
{
public:
  ros::NodeHandle n;
  ros::Subscriber robot_pose_sub, object_detection_sub;
  ros::Publisher desired_velocity_pub;
  ros::ServiceServer path_follower_server;
  ros::ServiceClient pure_rotation_srv, is_cell_occupied_srv, move_straight_srv, classification_srv;

  path_follower_v2()
  {
    //Initialisation parameters
    n.param<float>("/path_follower_v2/distance_to_destination_threshold", dest_threshold, 0.01);
    n.param<float>("/path_follower_v2/distance_to_jump_to_next_point", dest_to_next_point, 0.05);
    n.param<float>("/path_follower_v2/angle_P", a_P, 0.05);
    n.param<float>("/path_follower_v2/angular_velocity_saturation", desire_angular_sat, 0.05);
    n.param<float>("/path_follower_v2/angular_threshold_trust", angle_ref_max, pi/8);
    n.param<float>("/path_follower_v2/aver_linear_speed", aver_lin_vel, 0.0);
    n.param<float>("/path_follower_v2/discretization_length", discretize_length, 0.0);
    n.param<bool>("/path_follower_v2/following_point_mode", point_follower_mode, true);
    n.param<bool>("/path_follower_v2/mapping_mode", mapping_mode, true);

    robot_pose_sub = n.subscribe("/localization/kalman_filter/position_timed", 1, &path_follower_v2::position_callBack, this);
    object_detection_sub = n.subscribe("/vision/state", 1, &path_follower_v2::detection_callBack, this);

    desired_velocity_pub = n.advertise<geometry_msgs::Twist>("/desired_velocity", 1);

    pure_rotation_srv = n.serviceClient<robo7_srvs::PureRotation>("/kinematics/path_follower/pure_rotation");
    is_cell_occupied_srv = n.serviceClient<robo7_srvs::IsGridOccupied>("/occupancy_grid/is_occupied");
    move_straight_srv = n.serviceClient<robo7_srvs::MoveStraight>("/kinematics/path_follower/straight_move");
    classification_srv = n.serviceClient<robo7_srvs::FilterOn>("/object_filter/activate");

    path_follower_server = n.advertiseService("/kinematics/path_follower/path_follower_v2", &path_follower_v2::path_follower_Sequence, this);
  }

  void position_callBack(const robo7_msgs::the_robot_position::ConstPtr &msg)
  {
    the_robot_pose = *msg;
  }

  void detection_callBack(const robo7_msgs::detectedState::ConstPtr &msg)
  {
    the_objects_states = *msg;
  }

  bool path_follower_Sequence(robo7_srvs::PathFollower2::Request &req,
         robo7_srvs::PathFollower2::Response &res)
  {
    robo7_msgs::target_trajectory the_trajectory = req.trajectory;
    robo7_msgs::trajectory trajectory_array = req.traject;
    the_trajectory.number_of_path = static_cast<int>(the_trajectory.target_trajectory_points.size());

    // ROS_INFO("Starting to follow path");

    bool path_ended = false;

    if(trajectory_array.trajectory_points.size() == 0)
    {
      path_ended = true;
    }

    // robo7_msgs::wallPoint the_discretized_path = discretize_the_path( the_trajectory , discretize_length );
    robo7_msgs::wallPoint the_discretized_path = discretize_the_path2( trajectory_array , discretize_length );

    // ROS_INFO("Start moving with %d points", the_discretized_path.number);

    if(point_follower_mode&&!path_ended)
    {

      int index_point_following = 0;
      geometry_msgs::Vector3 point_following;

      if(true)
      {
        point_following = the_discretized_path.the_points[0];
        float x1 = the_robot_pose.position.linear.x;
        float y1 = the_robot_pose.position.linear.y;
        float x2 = point_following.x;
        float y2 = point_following.y;
        initial_angle = findangle( x1-x2 , y1-y2 );

        //First align the robot with the path start (aka call service)
        robo7_srvs::PureRotation::Request req1;
        robo7_srvs::PureRotation::Response res1;
        req1.desired_angle = initial_angle + pi;
        pure_rotation_srv.call(req1, res1);
      }

      geometry_msgs::Twist desire_vel;
      double time_prev = ros::Time::now().toSec();

      ros::Rate loop_rate(100);

      //Then make it follow the path
      while(!path_ended)
      {
        //Extract the position with the subscriber
        ros::spinOnce();

        float velocity_sign = 1;

        //Choose the point to follow
        index_point_following = point_to_follow( the_discretized_path , index_point_following );
        point_following = the_discretized_path.the_points[index_point_following];
        // ROS_INFO("point to follow %d over %d", index_point_following, the_discretized_path.number);

        if(mapping_mode&&!free_road( the_discretized_path , index_point_following ))
        {
          ROS_INFO("Road occupied , %lf", ros::Time::now().toSec());
          desire_vel.linear.x = 0;
          desire_vel.angular.z = 0;
          desired_velocity_pub.publish( desire_vel );
          ros::Rate r(1); r.sleep();
          ROS_INFO("Check cells , %lf", ros::Time::now().toSec());
          if(is_cell_occupied( the_robot_pose.position.linear.x , the_robot_pose.position.linear.y))
          {
            ROS_INFO("Let's back up");
            move_straight( 0.10 , true );
          }

          break;
        }

        double time_n = ros::Time::now().toSec();

        if(object_just_detected)
        {
          time_prev = time_n;
          object_just_detected = false;
        }

        ROS_INFO("%d & %d -> %lf & %lf", (the_objects_states.state.size()>0),(time_n - time_prev > 1), time_n, time_prev);
        if(mapping_mode&&(static_cast<int>(the_objects_states.state.size()>0))&&(time_n - time_prev > 1))
        {
          ros::Rate loop_rate(2.0);
          desire_vel.linear.x = 0;
          desire_vel.angular.z = 0;
          desired_velocity_pub.publish( desire_vel );
          loop_rate.sleep();
          ros::spinOnce();
          trigger_filtering_of_object();
          object_just_detected = true;
        }

        //Extract the angle difference out of this point to follow
        diff_angle = compute_diff_angle( point_following );
        // ROS_INFO("angle diff %lf", diff_angle);

        //Then update the linear speed
        if(sgn(diff_angle)*diff_angle > angle_ref_max)
        {
          desire_vel.linear.x = 0;
          desire_vel.angular.z = P_update( diff_angle );
          angle_ref_max = pi/8;
        }
        else
        {
          desire_vel.linear.x = velocity_sign * aver_lin_vel;
          desire_vel.angular.z = P_update( diff_angle );
          angle_ref_max = pi/6;
        }

        //Then check if the path came to an end
        if((index_point_following == the_discretized_path.number-1)
            &&(distance_to_point( the_discretized_path.the_points[index_point_following]) < dest_threshold))
          {
            path_ended = true;
            desire_vel.linear.x = 0;
            desire_vel.angular.z = 0;
          }

        //Then publish the speed
        desired_velocity_pub.publish( desire_vel );

        loop_rate.sleep();
      }

      //Republish an 0 speed
      desire_vel.linear.x = 0;
      desire_vel.angular.z = 0;
      desired_velocity_pub.publish( desire_vel );
    }

    res.success = path_ended;
  }


private:
  //Subscribers
  robo7_msgs::the_robot_position the_robot_pose;
  robo7_msgs::detectedState the_objects_states;

  //Controllers values
  float a_P;
  float desire_angular_sat;
  float angle_ref_max, diff_angle, initial_angle;
  float aver_lin_vel;

  //Which path follower we choose
  float dest_threshold, dest_to_next_point;
  float discretize_length;
  bool point_follower_mode;
  bool mapping_mode;
  bool object_detected;
  bool object_just_detected;

  int point_to_follow( robo7_msgs::wallPoint discretized_path_msg, int current_index)
  {
    int the_new_index = current_index;

    if(distance_to_point(discretized_path_msg.the_points[the_new_index]) < dest_to_next_point)
    {
      while((distance_to_point(discretized_path_msg.the_points[the_new_index]) < dest_to_next_point)
              &&(the_new_index < discretized_path_msg.number - 1))
        {
          the_new_index++;
        }
    }

    return the_new_index;
  }

  float distance_to_point( geometry_msgs::Vector3 the_point )
  {
    float x_r = the_robot_pose.position.linear.x;
    float y_r = the_robot_pose.position.linear.y;

    return sqrt(pow(x_r-the_point.x,2)+pow(y_r-the_point.y,2));
  }

  float P_update(float difference_angle)
  {
    float error = difference_angle;

    float angular_vel = (a_P * error);
    float angular_sat = desire_angular_sat;

    // ROS_INFO("angular vel = %lf and %lf", angular_vel, angular_sat);

    if(angular_vel > angular_sat) { angular_vel = angular_sat; }
    else if(angular_vel < -angular_sat) { angular_vel = -angular_sat; }

    return angular_vel;
  }

  float compute_diff_angle(geometry_msgs::Vector3 to_follow)
  {
    float x_r = the_robot_pose.position.linear.x;
    float y_r = the_robot_pose.position.linear.y;
    float a_r = the_robot_pose.position.angular.z;

    float x = (to_follow.x - x_r) * cos(a_r) + (to_follow.y - y_r) * sin(a_r);
    float y = - (to_follow.x - x_r) * sin(a_r) + (to_follow.y - y_r) * cos(a_r);

    return findangle(x, y);
  }

  float findangle(float x, float y)
  {
    if(x==0)
    {
      return pi*sgn(y);
    }
    else if((x<0)&&(y>0))
    {
      return atan(y/x) + pi;
    }
    else if ((x<0)&&(y<0))
    {
      return atan(y/x) - pi;
    }
    else
    {
      return atan(y/x);
    }
    // return atan(x/y);
  }

  robo7_msgs::wallPoint discretize_the_path( robo7_msgs::target_trajectory trajectory , float l )
  {
    robo7_msgs::wallPoint discretized_path;

    for(int i=0; i<trajectory.number_of_path; i++)
    {
      geometry_msgs::Vector3 aPoint;
      aPoint.x = trajectory.target_trajectory_points[i].end_point.linear.x;
      aPoint.y = trajectory.target_trajectory_points[i].end_point.linear.y;
      if(trajectory.target_trajectory_points[i].is_it_line)
      {
        aPoint.z = 0.2; //Distance before switching point
      }
      else
      {
        aPoint.z = 0.1; //Distance before switching for curves
      }

      discretized_path.number++;
      discretized_path.the_points.push_back(aPoint);
    }

    return discretized_path;
  }

  robo7_msgs::wallPoint discretize_the_path2( robo7_msgs::trajectory trajectory , float l )
  {
    robo7_msgs::wallPoint discretized_path;

    for(int i=0; i<static_cast<int>(trajectory.trajectory_points.size()); i++)
    {
      geometry_msgs::Vector3 aPoint;
      aPoint.x = trajectory.trajectory_points[i].point_coord.x;
      aPoint.y = trajectory.trajectory_points[i].point_coord.y;

      discretized_path.number++;
      discretized_path.the_points.push_back(aPoint);
    }

    return discretized_path;
  }

  float distance_point( geometry_msgs::Vector3 point1 , geometry_msgs::Vector3 point2 )
  {
    return sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
  }

  int sgn(float v)
  {
    if (v < 0) return -1;
    else if (v > 0) return 1;
    else return 0;
  }

  bool free_road( robo7_msgs::wallPoint discretized_path , int index )
  {
    robo7_srvs::IsGridOccupied::Request req1;
		robo7_srvs::IsGridOccupied::Response res1;
    for(int i=index; (i<discretized_path.number)&&(i<index+3); i++)
    {
      req1.x = discretized_path.the_points[i].x;
      req1.y = discretized_path.the_points[i].y;
      is_cell_occupied_srv.call(req1,res1);

      if(res1.occupancy == 1)
      {
        return false;
      }
    }
    return true;
  }

  void move_straight(float dist , bool move_back)
  {
    robo7_srvs::MoveStraight::Request req1;
    robo7_srvs::MoveStraight::Response res1;
    req1.desired_distance = dist;
    req1.move_backward = move_back;
    move_straight_srv.call(req1, res1);
  }

  void pure_rotation(float angle)
  {
    robo7_srvs::PureRotation::Request req1;
    robo7_srvs::PureRotation::Response res1;
    req1.desired_angle = angle;
    pure_rotation_srv.call(req1, res1);
  }

  bool is_cell_occupied( float x , float y )
  {
    robo7_srvs::IsGridOccupied::Request req1;
    robo7_srvs::IsGridOccupied::Response res1;
    req1.x = x;
    req1.y = y;
    is_cell_occupied_srv.call(req1,res1);

    ROS_INFO("cell occupied %lf ", res1.occupancy);

    return (res1.occupancy == 1.0);
  }

  void trigger_filtering_of_object()
  {
    if(the_objects_states.state[0] == 1)
    {
      //Turn left
      pure_rotation( -pi/6 );
    }
    else if(the_objects_states.state[0] == 2)
    {
      //Turn right
      pure_rotation( pi/6 );
    }

    //Then call the classifier service
    robo7_srvs::FilterOn::Request req1;
    robo7_srvs::FilterOn::Response res1;
    req1.time = 5.0;
    classification_srv.call(req1, res1);

    //Turn back to initial position
    if(the_objects_states.state[0] == 1)
    {
      //Turn right
      pure_rotation( pi/6 );
    }
    else if(the_objects_states.state[0] == 2)
    {
      //Turn left
      pure_rotation( -pi/6 );
    }
  }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_follower_v2");

    path_follower_v2 path_follower_v2_;

    ros::Rate loop_rate(control_frequency);

    ROS_INFO("Path Follower v2 running");

    ros::spin();

    return 0;
}
