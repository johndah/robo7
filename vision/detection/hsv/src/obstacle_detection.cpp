#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class ObjectDetection
{
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  ros::NodeHandle n;
  ros::Subscriber depth_points_sub;
  ros::Publisher obj_pos_pub;
  int camera_width = 640, camera_height = 480;

  ObjectDetection()
      : it_(nh_)
  {
    
    namedWindow("Original image");
    namedWindow("Filtered image");

    depth_points_sub = n.subscribe("/camera/depth_registered/points", 1, &ObjectDetection::depthCallBack, this);
    obj_pos_pub = n.advertise<geometry_msgs::Point>("/obstacle/pos", 5);
  }

  ~ObjectDetection()
  {
    destroyAllWindows();
  }

  void depthCallBack(const sensor_msgs::PointCloud2 &msg)
  {
    pCloud_cam = msg;
  }

  geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v)
  {
    // get width and height of 2D point cloud data
    int width = pCloud.width;
    int height = pCloud.height;

    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = v * pCloud.row_step + u * pCloud.point_step;

    // compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    // ROS_INFO("xx: %f", X);
    // ROS_INFO("yy: %f", Y);
    // ROS_INFO("zz: %f", Z);

    geometry_msgs::Point p;
    p.x = X;
    p.y = Y;
    p.z = Z;

    return p;
  }

  bool obstacleDetected()
  {
    int y = camera_height / 2;

    for (int x = 0; x < camera_width; x++)
    {
      p = pixelTo3DPoint(pCloud_cam, x, y);
      ROS_INFO("Depth: %f", p.z);
    }

    return false;
  }

private:
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::PointCloud2 pCloud_cam;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection");

  ObjectDetection od;

  while (ros::ok())
  {

    obstacleDetected()

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}