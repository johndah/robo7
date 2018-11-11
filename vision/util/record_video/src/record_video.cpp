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
  // image_transport::Publisher image_pub_;


  RecordVideo()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
      &RecordVideo::saveVideo, this);
    // image_pub_ = it_.advertise("/image_converter/output_image", 1);

    // cv::namedWindow(OPENCV_WINDOW);

    dir = "/home/jtao/out.avi";
    n.getParam("record_video/dir", dir);
    video.open(dir, VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, Size(640, 480));

    if (video.isOpened())
    {
      ROS_INFO("Initialize successfully!");
    }
    else
    {
      ROS_INFO("Initializion fail!");
    }
  }

  ~RecordVideo()
  {
    video.release();
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void saveVideo(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    video.write(cv_ptr->image);

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    // ROS_INFO("width: %d", cv_ptr->image.cols);
    // ROS_INFO("height: %d", cv_ptr->image.rows);

    cv::waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->image);
  }

private:

  cv::VideoWriter video;
  string dir;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "record_video");
  RecordVideo ic;

  ros::spin();

  return 0;
}
