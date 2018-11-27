#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class RecordVideo
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  RecordVideo()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
      &RecordVideo::saveVideo, this);
    // image_pub_ = it_.advertise("/image_converter/output_image", 1);

    // cv::namedWindow(OPENCV_WINDOW);

    dir = "/home/ras17/out.avi";
    n.getParam("record_depth_video/dir", dir);
    video.open(dir, VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, Size(640, 480));

    if (video.isOpened())
      ROS_INFO("Initialize successfully!");
    else
      ROS_INFO("Initializion fail!");
  }

  ~RecordVideo()
  {
    video.release();
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void saveVideo(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    Mat origImg = cv_ptr->image;

    Mat depthImage = origImg.clone();

    double min;
    double max;
    cv::minMaxIdx(depthImage, &min, &max);

    cv::Mat adjMap;
    // expand your range to 0..255. Similar to histEq();
    depthImage.convertTo(adjMap, CV_8UC1, 255 / (max-min), -min);

    cv::Mat falseColorsMap;
    applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);

    rectangle(falseColorsMap, Point(0, 230), Point(640, 250), Scalar(255,255,255), 2, 8, 0);

    cv::imshow("Filtered image", falseColorsMap);

    if (falseColorsMap.empty())
      ROS_INFO("empty image!");
    else
      video.write(falseColorsMap);

    cv::waitKey(3);

  }

private:

  cv::VideoWriter video;
  string dir;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "record_depth_video");
  RecordVideo ic;

  ros::spin();

  return 0;
}
