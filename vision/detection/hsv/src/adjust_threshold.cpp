#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class HSVthreshold
{
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

  	HSVthreshold()
  	  : it_(nh_)
  	{
  		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
  			                       &HSVthreshold::filter, this);

  		namedWindow("Original image");
  		namedWindow("Filtered image");

  		h_min = 0;
  		h_max = 180;
  		s_min = 0;
  		s_max = 255;
  		v_min = 0;
  		v_max = 255;

  		createTrackbar("H min", "Filtered image", &h_min, 180, HSVthreshold::on_trackbar);
  		createTrackbar("H max", "Filtered image", &h_max, 180, HSVthreshold::on_trackbar);
  		createTrackbar("S min", "Filtered image", &s_min, 255, HSVthreshold::on_trackbar);
  		createTrackbar("S max", "Filtered image", &s_max, 255, HSVthreshold::on_trackbar);
  		createTrackbar("V min", "Filtered image", &v_min, 255, HSVthreshold::on_trackbar);
  		createTrackbar("V max", "Filtered image", &v_max, 255, HSVthreshold::on_trackbar);
  	}

  	~HSVthreshold()
  	{
  		destroyAllWindows();
  	}

  	void filter(const sensor_msgs::ImageConstPtr& msg)
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  		imshow("Original image", cv_ptr->image);
      cv::waitKey(3);

      Mat hsv;
      Mat chanel[3];
      Mat ch_result[3];
      Mat result;

      cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);
      split(hsv, chanel);

      threshold(chanel[0],ch_result[0],h_min,255,THRESH_BINARY);
      threshold(chanel[0],ch_result[1],h_max,255,THRESH_BINARY_INV);
      bitwise_and(ch_result[0],ch_result[1],ch_result[0]);

      threshold(chanel[1],ch_result[1],s_min,255,THRESH_BINARY);
      threshold(chanel[1],ch_result[2],s_max,255,THRESH_BINARY_INV);
      bitwise_and(ch_result[1],ch_result[2],ch_result[1]);
      bitwise_and(ch_result[0],ch_result[1],ch_result[0]);

      threshold(chanel[2],ch_result[1],v_min,255,THRESH_BINARY);
      threshold(chanel[2],ch_result[2],v_max,255,THRESH_BINARY_INV);
      bitwise_and(ch_result[1],ch_result[2],ch_result[1]);
      bitwise_and(ch_result[0],ch_result[1],ch_result[0]);

      erode(ch_result[0],result, Mat(5,5,CV_8UC1),Point(-1,-1),1);

      imshow("Filtered image", result);
  	}

  	static void on_trackbar(int, void*)
  	{
  	}


private:
	cv_bridge::CvImagePtr cv_ptr;
	int h_min;
	int h_max;
	int s_min;
	int s_max;
	int v_min;
	int v_max;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "adjust_threshold");

	HSVthreshold node;

	ros::spin();

	return 0;

}