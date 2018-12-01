#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class FeedVideo
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  image_transport::ImageTransport it_;
  // image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;


  FeedVideo()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    // image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
      // &RecordVideo::saveVideo, this);
    image_pub_ = it_.advertise("/camera/rgb/image_rect_color", 1);

    cv::namedWindow(OPENCV_WINDOW);

    dir = "/home/jtao/out.avi";
    n.getParam("feed_video/dir", dir);
  }

  ~FeedVideo()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void feedImage()
  {
    cv::VideoCapture cap(dir);

    while(1){
      cv::Mat image;
      cap >> image;

      char c=(char)cv::waitKey(25);
      if (c==27)
        break;

      if (!image.empty()){
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub_.publish(img_msg);

        cv::imshow(OPENCV_WINDOW, image);
        cv::waitKey(3);
      }
    }

    cap.release();

  }

private:

  string dir;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feed_video");
  FeedVideo ic;
  ic.feedImage();

  ros::spin();

  return 0;
}
