#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
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
  image_transport::Subscriber depth_image_sub;

  ros::NodeHandle n;

  ros::Publisher obstale_detect_pub;

  ObjectDetection()
      : it_(nh_)
  {
    namedWindow("Original image");
    namedWindow("Filtered image");
    frame_acc = 0;

    depth_image_sub = it_.subscribe("/camera/depth/image_raw", 1, &ObjectDetection::depthImageCallBack, this);

    obstale_detect_pub = n.advertise<std_msgs::Bool>("/obstacle/flag", 1);

  }

  ~ObjectDetection()
  {
    destroyAllWindows();
  }

  void depthImageCallBack(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat depthImage = cv_ptr->image;

    depthImage.setTo(2000, depthImage < 10);
    double min;
    double max;

    medianBlur ( depthImage, depthImage, 5);
    cv::minMaxIdx(depthImage, &min, &max);

    // depthImage = threshold(depthImage, depthImage, 10, max, THRESH_BINARY);

    ROS_INFO("min: %f", min);
    ROS_INFO("max: %f", max);


    flag.data = false;

    if (min < 150)
    {
      flag.data = true;
      frame_acc++;
      ROS_INFO("frame_acc: %d", frame_acc);
    }

    if (flag.data == false)
    {
      frame_acc = 1;
    }

    if (frame_acc >= 8)
    {
      flag.data = true;
    }
    else
    {
      flag.data = false;
    }


    obstale_detect_pub.publish(flag);
    cv::Mat adjMap;
    // expand your range to 0..255. Similar to histEq();
    depthImage.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min);

// this is great. It converts your grayscale image into a tone-mapped one,
// much more pleasing for the eye
// function is found in contrib module, so include contrib.hpp
// and link accordingly
    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);

      cv::imshow("Filtered image", falseColorsMap);

      // Mat element = getStructuringElement(0, Size(5, 5), Point(5, 5));
      // morphologyEx(falseColorsMap, falseColorsMap, 2, element);

    // depthImage.convertTo(depthImage, CV_32F);

    // cv::imshow("Filtered image", depthImage);
    // cv::imshow("depth_image", cv_ptr->image);

    // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    // cv::normalize(cv_ptr->image, cv_ptr->image, 0, 255, cv::NORM_MINMAX);
    imshow("Original image", cv_ptr->image);
    cv::waitKey(3);
  }

  // void depthCallBack(const sensor_msgs::PointCloud2 &msg)
  // {
  //   pCloud_cam = msg;
  // }
  //
  // geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v)
  // {
  //   // get width and height of 2D point cloud data
  //   int width = pCloud.width;
  //   int height = pCloud.height;
  //
  //   // Convert from u (column / width), v (row/height) to position in array
  //   // where X,Y,Z data starts
  //   int arrayPosition = v * pCloud.row_step + u * pCloud.point_step;
  //
  //   // compute position in array where x,y,z data start
  //   int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  //   int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  //   int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
  //
  //   float X = 0.0;
  //   float Y = 0.0;
  //   float Z = 0.0;
  //
  //   memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  //   memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  //   memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));
  //
  //   // ROS_INFO("xx: %f", X);
  //   // ROS_INFO("yy: %f", Y);
  //   // ROS_INFO("zz: %f", Z);
  //
  //   geometry_msgs::Point p;
  //   p.x = X;
  //   p.y = Y;
  //   p.z = Z;
  //
  //   return p;
  // }
  //
  // bool obstacleDetected()
  // {
  //   //geometry_msgs::Point p;
  //
  //   int y = camera_height / 2;
  //
  //   for (int x = 0; x < camera_width; x++)
  //   {
  //     p = pixelTo3DPoint(pCloud_cam, x, y);
  //     ROS_INFO("Depth: %f", p.z);
  //   }
  //
  //   return false;
  // }

private:
  cv_bridge::CvImagePtr cv_ptr;
  int frame_acc;
  std_msgs::Bool flag;
  // sensor_msgs::PointCloud2 pCloud_cam;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "obstacle_detection");

  // int control_frequency = 10;
  // ros::Rate loop_rate(control_frequency);

  ObjectDetection od;

  // while (ros::ok())
  // {
  //
  //   od.obstacleDetected();
  //
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  ros::spin();

  return 0;
}
