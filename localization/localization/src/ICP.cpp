//Input all the libraries needed
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <phidgets/motor_encoder.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <robo7_msgs/XY_coordinates.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include </usr/include/eigen3/Eigen/Dense>
#include <stdlib.h>
using Eigen::Matrix4f;

// std includes
#include <limits>
#include <iostream>
#include <fstream>

// Boost includes
#include <stdio.h>
#include <stdlib.h>

using namespace std;

// Control @ 10 Hz
double control_frequency = 100.0;


class ICP
{
public:
  ros::NodeHandle n;
  ros::Subscriber lidar_cloud;
  ros::Subscriber robot_position;
  ros::Subscriber map_cloud;
  ros::Publisher lidar_position;
  ros::Publisher matrix_values;

  ICP()
  {
    pi = 3.14159265358979323846;
    length = 0;
    length_wall;

    robot_position = n.subscribe("/localization/Pos", 10, &ICP::position_callBack, this);
    lidar_cloud = n.subscribe("/scan_to_coordinates/point_cloud_coordinates", 10, &ICP::point_cloud_callBack, this);
    map_cloud = n.subscribe("/maze_map/wall_coordinates", 1, &ICP::wall_cloud_callBack, this);
    lidar_position = n.advertise<geometry_msgs::Twist>("Pos", 10);
    matrix_values = n.advertise<robo7_msgs::XY_coordinates>("/matrix_values", 10);

  }

  void point_cloud_callBack(const robo7_msgs::XY_coordinates::ConstPtr &msg)
  {
      length = msg->length;
      converted_X_coordinates = msg->X_coordinates;
      converted_Y_coordinates = msg->Y_coordinates;
  }

  void wall_cloud_callBack(const robo7_msgs::XY_coordinates::ConstPtr &msg)
  {
      length_wall = msg->length;
      wall_X_coordinates = msg->X_coordinates;
      wall_Y_coordinates = msg->Y_coordinates;
  }

  void position_callBack(const geometry_msgs::Twist::ConstPtr &msg)
  {
      current_x = msg->linear.x;
      current_y = msg->linear.y;
      current_angle = msg->angular.z;
  }

  void updateICP_lib()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the CloudIn data
    cloud_source->width    = length;
    cloud_source->height   = 1;
    cloud_source->is_dense = false;
    cloud_source->points.resize (cloud_source->width * cloud_source->height);
    for (size_t i = 0; i < converted_X_coordinates.size(); ++i)
    {
      cloud_source->points[i].x = converted_X_coordinates[i];
      cloud_source->points[i].y = converted_Y_coordinates[i];
      cloud_source->points[i].z = 0;
    }

    cloud_target->width    = length_wall;
    cloud_target->height   = 1;
    cloud_target->is_dense = false;
    cloud_target->points.resize (cloud_target->width * cloud_target->height);
    for (size_t i = 0; i < wall_X_coordinates.size(); ++i)
    {
      cloud_target->points[i].x = wall_X_coordinates[i];
      cloud_target->points[i].y = wall_Y_coordinates[i];
      cloud_target->points[i].z = 0;
    }


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource (cloud_source);
    icp.setInputTarget (cloud_target);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (1);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
    // Perform the alignment
    icp.align (cloud_source_registered);
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    transformation = icp.getFinalTransformation ();

    geometry_msgs::Twist twist_msg;

    mat_val = vector<float>(1, 0);

    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        mat_val.push_back(transformation(i,j));
      }
    }

    robo7_msgs::XY_coordinates point_XY;
    point_XY.length = mat_val.size();
    point_XY.trueX_length = 0;
    point_XY.trueY_length = 0;
    point_XY.X_coordinates = mat_val;
    matrix_values.publish(twist_msg);
  }

  void updateICP()
  {
    int k = 0;
    wall_X_centered = vector<float>(converted_X_coordinates.size(), 0);
    wall_Y_centered = vector<float>(converted_X_coordinates.size(), 0);
    X_centered = vector<float>(converted_X_coordinates.size(), 0);
    Y_centered = vector<float>(converted_X_coordinates.size(), 0);
    while((k<max_iter)&&(err_val > dist_threshold))
    {
      closest_ind = findAllClosePoints(X_moved, Y_moved, wall_X_coordinates, wall_Y_coordinates);
      meanValues();
      centered_point();
      transform();
      move_Src_points();
      final_x_trans += trans_x;
      final_y_trans += trans_y;
      final_theta_rot += rot_theta;
      err_val = errorComputation(X_moved, Y_moved, wall_X_close, wall_Y_close);
      k++;
    }
  }

  void meanValues()
  {
    mean_Xsrc = meanListValue(X_moved);
    mean_Ysrc = meanListValue(Y_moved);
    mean_Xtgr = meanListValue(wall_X_close);
    mean_Ytgr = meanListValue(wall_Y_close);
  }

  void centered_point()
  {
    for(int i=0; i<X_moved.size(); i++)
    {
      wall_X_centered[i] = wall_X_close[i]/mean_Xtgr;
      wall_Y_centered[i] = wall_Y_close[i]/mean_Ytgr;
      X_centered[i] = X_moved[i]/mean_Xsrc;
      Y_centered[i] = Y_moved[i]/mean_Ysrc;
    }
  }

  void transform()
  {
    trans_x = mean_Xsrc - mean_Xtgr;
    trans_y = mean_Ysrc - mean_Ytgr;
    rot_theta = 0;
    for(int i=0; i<X_moved.size(); i++)
    {
      rot_theta += computeRotation(X_centered[i], Y_centered[i], wall_X_centered[i], wall_Y_centered[i])/length;
    }
  }

  void move_Src_points()
  {
    for(int i=0; i<X_moved.size(); i++)
    {
      X_moved = X_centered[i]*cos(rot_theta) - Y_centered * sin(rot_theta) + mean_Xsrc;
      Y_moved = X_centered[i]*cos(rot_theta) + Y_centered * sin(rot_theta) + mean_Ysrc;
    }
  }



private:
  //Localization main parameters
  float current_x;
  float current_y;
  float current_angle;

  //ICP parameters
  float dist_threshold;
  int max_iter;
  //Find close points
  vector<float> wall_X_close;
  vector<float> wall_Y_close;
  //Centered Vectors
  vector<float> wall_X_centered;
  vector<float> wall_Y_centered;
  vector<float> X_centered;
  vector<float> Y_centered;
  //Moved vectors after translation/rotation
  vector<float> X_moved;
  vector<float> Y_moved;
  //Translatio/rotation after an iteration
  float trans_x;
  float trans_y;
  float rot_theta;

  //ICP results
  float final_x_trans;
  float final_y_trans;
  float final_theta_rot;


  //ICP library
  Eigen::Matrix4f transformation;
  pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
  vector<float> mat_val;




  //Lidar function parameters
  int length;
  vector<float> converted_X_coordinates;
  vector<float> converted_Y_coordinates;

  //Walls parameters
  int length_wall;
  vector<float> wall_X_coordinates;
  vector<float> wall_Y_coordinates;

  string _map_file;
  string _map_frame;
  string _map_topic;

  //other parameters
  float pi;


  float pointDistance(float x1, float y1, float x2, float y2)
  {
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
  }

  int findCloserPoint_index(float x1, float y1, vector<float> X2, vector<float> Y2)
  {
    int ind = 0;
    float least = pointDistance(x1, y1, X2[0], Y2[0]);
    for(int i=1; i<X1.size(); i++)
    {
      if(pointDistance(x1, y1, X2[i], Y2[i]) < least)
      {
        ind = i;
        least = pointDistance(x1, y1, X2[i], Y2[i]);
      }
    }
    return i;
  }

  vector<int> findAllClosePoints(vector<float> X1, vector<float> Y1, vector<float> X2, vector<float> Y2)
  {
    vector<int> ind_list;
    int mid_ind;
    ind_list = vector<int>(X1.size(), 0);
    wall_X_close = vector<int>(X1.size(), 0);
    wall_Y_close = vector<int>(X1.size(), 0);
    for(int i=0; i<X1.size(); i++)
    {
      mid_ind = findCloserPoint_index(X1[i], Y1[i], X2, Y2);
      ind_list[i] = mid_ind;
      wall_X_close[i] = wall_X_coordinates[mid_ind];
      wall_Y_close[i] = wall_Y_coordinates[mid_ind];
    }
    return ind_list;
  }

  float meanListValue(vector<int> V)
  {
    float mean = 0;
    int l = V.size();
    for(int i=0; i<l; i++)
    {
      mean += V[i]/l;
    }
    return mean;
  }

  float computeRotation(float x1, float y1, float x2, float y2)
  {
    if((x1!=0)&&(x2!=0))
    {
      float xy1 = y1/x1;
      float xy2 = y2/x2;
      if(xy1==xy2){return 0;}
      else{float ang = atan((xy1+xy2)/1-xy1*xy2);}
      if((xy1*xy2>1)&&(xy1>0)){return wrapAngle(ang+pi);}
      else if((xy1*xy2>1)&&(xy1<0)){return wrapAngle(ang-pi);}
      else{return wrapAngle(ang);}
    }
    else if((x1==0)&&(x2==0))
    {
      if(y1*y2>0){return 0;}
      else{return pi/2;}
    }
    else if((x1==0))
    {
      return wrapAngle(-atan(y2/x2));
    }
    else
    {
      return wrapAngle(atan(y1/x1));
    }


  }

  float wrapAngle( double angle )
  {
    float twoPi = 2.0 * pi;
    return angle - twoPi * floor( angle / twoPi );
  }

  float errorComputation(vector<float> X1, vector<float> Y1, vector<float> X2, vector<float> Y2)
  {
    float err;
    for(int i=0; i<X1.size(); i++)
    {
      err += pointDistance(X1[i], Y1[i], X2[i], Y2[i])/X1.size();
    }
  }
};







int main(int argc, char **argv)
{
    ros::init(argc, argv, "ICP");

    ICP _ICP;

    ros::Rate loop_rate(control_frequency);

    while(_ICP.n.ok())
    {
        _ICP.updateICP();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
