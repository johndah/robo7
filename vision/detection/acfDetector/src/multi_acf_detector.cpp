#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <acf/ACF.h>
#include "acfDetector/detectedObj.h"

using namespace std;
using namespace cv;
namespace acf {
    class Detector;
    class ObjectDetector;
}

using ObjectDetectorPtr = std::shared_ptr<acf::ObjectDetector>;
using AcfPtr = std::shared_ptr<acf::Detector>;
using RectVec = std::vector<cv::Rect>;

class ACFdetector
{
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
  // image_transport::Publisher obj_img_pub;

  ros::NodeHandle n;
  ros::Subscriber depth_points_sub;
  // ros::Publisher obj_pos_pub;

  ros::Publisher obj_detected_pub;

  	ACFdetector()
  	  : it_(nh_)
  	{
  		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
  			                       &ACFdetector::detection, this);

      // obj_img_pub = it_.advertise("/vision/object/img", 1);

  		namedWindow("Detected image");
      // namedWindow("Cropped object image");

      depth_points_sub = n.subscribe("/camera/depth_registered/points", 1, &ACFdetector::depthCallBack, this);
      // obj_pos_pub = n.advertise<geometry_msgs::Point>("/vision/object/pos", 5);
      obj_detected_pub = n.advertise<acfDetector::detectedObj>("/vision/object", 1);


      std::string y_Model;
      n.param<string>("/multi_acf_detector/y_Model", y_Model, "/home/jtao/CLionProjects/lab1/model_color_mag.cpb");
      y_detector = std::make_shared<acf::Detector>(y_Model);
      if (y_detector.get() && y_detector->good()) {
        y_detector->setDoNonMaximaSuppression(true);
      }

      std::string g_Model;
      n.param<string>("/multi_acf_detector/g_Model", g_Model, "/home/jtao/CLionProjects/lab1/model_color_mag.cpb");
      g_detector = std::make_shared<acf::Detector>(g_Model);
      if (g_detector.get() && g_detector->good()) {
        g_detector->setDoNonMaximaSuppression(true);
      }

      std::string o_Model;
      n.param<string>("/multi_acf_detector/o_Model", o_Model, "/home/jtao/CLionProjects/lab1/model_color_mag.cpb");
      o_detector = std::make_shared<acf::Detector>(o_Model);
      if (o_detector.get() && o_detector->good()) {
        o_detector->setDoNonMaximaSuppression(true);
      }

    }

  	~ACFdetector()
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

    cv::Rect enlargeBBX(cv::Rect o, int cols, int rows)
    {
      cv::Rect large_bbx;

      if(o.width >= rows/2)
      {
        large_bbx.y = 0;
        large_bbx.height = rows;
        large_bbx.width = rows;
        int x_tl = o.x - rows/2;
        if(x_tl < 0)
          large_bbx.x = 0;
        else
          large_bbx.x = x_tl;
      }
      else
      {
        int x_tl = o.x - o.width / 2;
        int y_tl = o.y - o.height / 2;
        int x_br = o.x + 3 * o.width / 2;
        int y_br = o.y + 3 * o.height / 2;

        if (x_tl >= 0)
          large_bbx.x = x_tl;
        else
          large_bbx.x = 0;
        if (y_tl >= 0)
          large_bbx.y = y_tl;
        else
          large_bbx.y = 0;
        if (x_br >= cols)
          large_bbx.x = cols - 2 * o.width;
        if (y_br >= rows)
          large_bbx.y = rows - 2 * o.height;

        large_bbx.width = 2 * o.width;
        large_bbx.height = 2 * o.height;
      }

      // ROS_INFO("x: %d", o.x);
      // ROS_INFO("y: %d", o.y);
      // ROS_INFO("width: %d", o.width);
      // ROS_INFO("height: %d", o.height);

      // ROS_INFO("-------------------------------");

      // ROS_INFO("x: %d", large_bbx.x);
      // ROS_INFO("y: %d", large_bbx.y);
      // ROS_INFO("width: %d", large_bbx.width);
      // ROS_INFO("height: %d", large_bbx.height);

      // ROS_INFO("-------------------------------");

      return large_bbx;
    }

    void applyDetector(AcfPtr detector, cv::Mat origImg)
    {
      std::vector<double> scores;
      std::vector<cv::Rect> objects;

      (*detector)(origImg, objects, &scores);

      for (const auto& o : objects)
      {
        acfDetector::detectedObj object_pub;

        cv::rectangle(resultImg, o, { 0, 255, 0 }, 2, 8);
        Point center = (o.tl()+o.br())/2;
        circle(resultImg, center, 2, Scalar(0,0,255), 2, 8, 0);

        // ROS_INFO("x: %d", o.x);
        // ROS_INFO("y: %d", o.y);
        ROS_INFO("width: %d", o.width);
        ROS_INFO("height: %d", o.height);

        // make sure the bbx don't out of the image
        if (o.x >= 0 && o.y >= 0 && (o.x + o.width) <= origImg.cols && (o.y + o.height) <= origImg.rows)
        {

          cv::Rect large_o = ACFdetector::enlargeBBX(o, origImg.cols, origImg.rows);
          cv::Mat obj_img;
          obj_img = origImg(large_o);

          // imshow("Cropped object image", obj_img);

          sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", obj_img).toImageMsg();

          // obj_img_pub.publish(img_msg);
          object_pub.img = *img_msg;

          // get the pos of the object
          geometry_msgs::Point pos;

          if (pCloud_cam.width != 0)
          {
            pos = pixelTo3DPoint(pCloud_cam, center.x, center.y);
            if (isnan(pos.x))
            {
              // get other 4 pixels around center
              for (int j=-6; j<7; j=j+4)
              {
                pos = pixelTo3DPoint(pCloud_cam, center.x + j, center.y);
                if (!isnan(pos.x))
                  break;

              for (int k=-6; j<7; k=k+4)
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

          // obj_pos_pub.publish(pos);
          object_pub.pos = pos;
          std_msgs::String color;
          color.data = 'red';
          object_pub.color = color;
          object_pub.objClass = 1;

          obj_detected_pub.publish(object_pub);
        }
      }
    }

  	void detection(const sensor_msgs::ImageConstPtr& msg)
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      cv::Mat origImg;
      origImg = cv_ptr->image;

      if (origImg.empty()) {ROS_INFO("empty image !!!!!!");}
      else
      {
        resultImg = origImg.clone();

        ACFdetector::applyDetector(y_detector, origImg);
        ACFdetector::applyDetector(g_detector, origImg);
        ACFdetector::applyDetector(o_detector, origImg);

        imshow("Detected image", resultImg);
        // cv::waitKey(10);
      }

  	}



private:
	cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::PointCloud2 pCloud_cam;

  AcfPtr y_detector;
  AcfPtr g_detector;
  AcfPtr o_detector;

  cv::Mat resultImg;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "acf_detector");

	ACFdetector node;

  ros::Rate loop_rate(2);

	ros::spin();

	return 0;

}
