#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "robo7_msgs/detectedObstacle.h"

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
    namedWindow("Filtered image");
    frame_acc = 0;

    depth_image_sub = it_.subscribe("/camera/depth/image_raw", 1, &ObjectDetection::depthImageCallBack, this);
    // obstale_detect_pub = n.advertise<std_msgs::Bool>("/obstacle/flag", 1);
    obstale_detect_pub = n.advertise<robo7_msgs::detectedObstacle>("vision/obstacle", 1);

    // Parameter setting
    num = 0;
    distThre = 10;
    depthThre = 140;
    numThre = 5000;
  }

  ~ObjectDetection()
  {
    destroyAllWindows();
  }

  void obstacle_Pos_estimation(Mat img, float &mean, int &num, float &depth, int &left, int &right){
    float sum = 0;
    float pixelValue;
    num = 0;
    for (int i=0; i<=img.rows; i++){
      for (int j=0; j<=img.cols; j++){
        pixelValue = img.at<unsigned short>(i, j);

        if (pixelValue == 0)
          continue;
        else{
          sum = sum + pixelValue;
          num = num + 1;
        }
      }
    }
    mean = sum / num;

    // img.setTo(0, img > 230);
    // medianBlur(img, img, 5);

    int numObstcale = 0;
    left = -1;
    right = -1;
    sum = 0;
    for (int j=0; j<=img.cols; j++){
        pixelValue = img.at<unsigned short>(10, j);
        if (pixelValue == 0 || pixelValue > 230){
          if (right == -2)
            right = j;
          continue;
        }

        else{
          sum = sum + pixelValue;
          numObstcale = numObstcale + 1;

          if (left == -1)
            left = j;

          right = -2;
        }
      }
    if (right == -2)
      right = img.cols;
    depth = sum / numObstcale;

    ROS_INFO("depth: %f", depth);
    ROS_INFO("left: %d", left);
    ROS_INFO("right: %d", right);
  }


  void depthImageCallBack(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat origImg = cv_ptr->image;

    Mat depthImage = origImg.clone();

    // Preprocess the depth image
    depthImage.setTo(0, depthImage < 10);
    depthImage.setTo(100, depthImage > 230);
    medianBlur ( depthImage, depthImage, 5);

    // ROS_INFO("min: %f", min);
    // ROS_INFO("max: %f", max);

    // flag.data = false;

    // // if (min < 150)
    // // {
    // //   flag.data = true;
    // //   frame_acc++;
    // //   // ROS_INFO("frame_acc: %d", frame_acc);
    // // }
    // //
    // // if (flag.data == false)
    // // {
    // //   frame_acc = 1;
    // // }
    // //
    // // if (frame_acc >= 8)
    // // {
    // //   flag.data = true;
    // // }
    // // else
    // // {
    // //   flag.data = false;
    // // }
    // obstale_detect_pub.publish(flag);


    // detect obstacle
    Mat cropedImage = origImg(Rect(0, 260, 640, 20));
    float mean, depth;
    int num, left, right;
    obstacle_Pos_estimation(cropedImage, mean, num, depth, left, right);
    ROS_INFO("mean value: %f", mean);
    ROS_INFO("num of pixels: %d", num);

    if ((right - left) < distThre)
      ROS_INFO("Obstacle in front!!");

    if (num < numThre)
      ROS_INFO("Black side in front!!");


    // Visualize depth image
    cv::Mat adjMap;
    double min;
    double max;
    cv::minMaxIdx(depthImage, &min, &max);
    // expand your range to 0..255. Similar to histEq();
    depthImage.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min);
    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);
    rectangle(falseColorsMap, Point(0, 260), Point(640, 280), Scalar(255,255,255), 2, 8, 0);

    if (left != -1 && right != -1){
      circle(falseColorsMap, Point(left, 270), 2, Scalar(255,255,255), 2, 8, 0);
      circle(falseColorsMap, Point(right, 270), 2, Scalar(255,255,255), 2, 8, 0);
    }

    cv::imshow("Filtered image", falseColorsMap);
    waitKey(2);



    // float depth, leftDist, rightDist;
    // obstacle_Pos_estimation(cropedImage, depth, leftDist, rightDist);


    // num = num + 1;
    // if (num == 20){
    //   cout << cropedImage << endl;
    //   cout << "------------------------------------" << endl;
    //   num = 0;
    //   cout << "mean value:" << endl;
    //   cout << meanValue(cropedImage) << endl;
    //   cout << "------------------------------------" << endl;
    // }




      // Mat element = getStructuringElement(0, Size(5, 5), Point(5, 5));
      // morphologyEx(falseColorsMap, falseColorsMap, 2, element);

    // depthImage.convertTo(depthImage, CV_32F);

    // cv::imshow("Filtered image", depthImage);
    // cv::imshow("depth_image", cv_ptr->image);

    // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    // cv::normalize(cv_ptr->image, cv_ptr->image, 0, 255, cv::NORM_MINMAX);

  }



private:
  cv_bridge::CvImagePtr cv_ptr;
  int frame_acc;
  std_msgs::Bool flag;
  // sensor_msgs::PointCloud2 pCloud_cam;
  int num;
  int numThre;
  float distThre;
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
