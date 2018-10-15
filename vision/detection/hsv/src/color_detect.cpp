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

class HSVthreshold
{
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

  ros::NodeHandle n;
  ros::Subscriber depth_points_sub;
  ros::Publisher obj_pos_pub;

  	HSVthreshold()
  	  : it_(nh_)
  	{
  		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
  			                       &HSVthreshold::filter, this);

  		namedWindow("Original image");
  		namedWindow("Filtered image");

      depth_points_sub = n.subscribe("/camera/depth_registered/points", 1, &HSVthreshold::depthCallBack, this);
      obj_pos_pub = n.advertise<geometry_msgs::Point>("/object/pos", 5);

    }

  	~HSVthreshold()
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
      int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

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

  	void filter(const sensor_msgs::ImageConstPtr& msg)
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // binary image filtered by color threshold
      Mat result = HSVthreshold::colorFilter(cv_ptr->image, 40-1, 72, 84-1, 255, 54-1, 146, 2, 6);

      // find contours
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      findContours(result, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
       
      std::vector<Rect> boundRect(contours.size()); 

      // draw rectangle
      Mat origImg = cv_ptr->image;
      geometry_msgs::Point pos;

      for (int i=0; i<contours.size(); i++)
      {
        if (contours[i].size() > 80)
        {
          ROS_INFO("object size: %d", contours[i].size());

          boundRect[i] = boundingRect(contours[i]);
          rectangle(origImg, boundRect[i].tl(), boundRect[i].br(), Scalar(0,0,255), 2, 8, 0);

          Point center = (boundRect[i].tl()+boundRect[i].br())/2;
          circle(origImg, center, 2, Scalar(0,0,255), 2, 8, 0);

          

          if (pCloud_cam.width != 0)
          {
            pos = pixelTo3DPoint(pCloud_cam, center.x, center.y);
            if (isnan(pos.x))
            {
              // get other 4 pixels around center
              for (int j=-1; j<2; j=j+2)
              {
                pos = pixelTo3DPoint(pCloud_cam, center.x + j, center.y);
                if (!isnan(pos.x))
                  break;

                for (int k=-1; j<2; k=k+2)
                {
                  pos = pixelTo3DPoint(pCloud_cam, center.x, center.y + k);

                  if (!isnan(pos.x))
                    break;
                }
                if (!isnan(pos.x))
                  break;
              }
            }

            ROS_INFO("center x: %d", center.x);
            ROS_INFO("center y: %d", center.y);
            ROS_INFO("final_x: %f", pos.x);
            ROS_INFO("final_y: %f", pos.y);
            ROS_INFO("final_z: %f", pos.z);
          }

        }
      }

      obj_pos_pub.publish(pos);
      imshow("Original image", origImg);
      cv::waitKey(3);

      imshow("Filtered image", result);
  	}

    Mat colorFilter(Mat src, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max, int opening_size, int closing_size)
    {

      Mat hsv;
      Mat chanel[3];
      Mat ch_result[3];
      Mat result;

      cvtColor(src, hsv, CV_BGR2HSV);
      split(hsv, chanel);

      threshold(chanel[0],ch_result[0],h_min-1,255,THRESH_BINARY);
      threshold(chanel[0],ch_result[1],h_max,255,THRESH_BINARY_INV);
      bitwise_and(ch_result[0],ch_result[1],ch_result[0]);

      threshold(chanel[1],ch_result[1],s_min-1,255,THRESH_BINARY);
      threshold(chanel[1],ch_result[2],s_max,255,THRESH_BINARY_INV);
      bitwise_and(ch_result[1],ch_result[2],ch_result[1]);
      bitwise_and(ch_result[0],ch_result[1],ch_result[0]);

      threshold(chanel[2],ch_result[1],v_min-1,255,THRESH_BINARY);
      threshold(chanel[2],ch_result[2],v_max,255,THRESH_BINARY_INV);
      bitwise_and(ch_result[1],ch_result[2],ch_result[1]);
      bitwise_and(ch_result[0],ch_result[1],ch_result[0]);

      // erode(ch_result[0],result, Mat(7,7,CV_8UC1),Point(-1,-1),1);

      Mat element = getStructuringElement(0, Size(2*opening_size+1, 2*opening_size+1), Point(opening_size, opening_size));
      morphologyEx(ch_result[0], result, 2, element);

      element = getStructuringElement(0, Size(2*closing_size+1, 2*closing_size+1), Point(closing_size, closing_size));
      morphologyEx(result, result, 3, element);

      return result;
    }


private:
	cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::PointCloud2 pCloud_cam;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "color_detect");

	HSVthreshold node;

	ros::spin();

	return 0;

}