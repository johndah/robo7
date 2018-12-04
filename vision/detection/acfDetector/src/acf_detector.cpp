#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>

#include <acf/ACF.h>
#include "acfDetector/detectedObj.h"

#include "robo7_msgs/detectedState.h"

using namespace std;
using namespace cv;
using namespace cv::ml;
namespace acf {
    class Detector;
    class ObjectDetector;
}

using ObjectDetectorPtr = std::shared_ptr<acf::ObjectDetector>;
using AcfPtr = std::shared_ptr<acf::Detector>;
using RectVec = std::vector<cv::Rect>;

// Load the svm color classifier
int width = 80, height = 80;
std::string svmModel = "/home/ras17/catkin_ws/src/robo7/vision/detection/acfDetector/model/SVMmodel.xml";
Ptr<SVM>svm = ml::SVM::load(svmModel);

class ACFdetector
{
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
  // image_transport::Publisher obj_img_pub;

  ros::NodeHandle n;
  ros::Subscriber depth_points_sub;
  ros::Subscriber classify_flag_sub;
  // ros::Publisher obj_pos_pub;

  ros::Publisher obj_detected_pub;
  ros::Publisher obj_state_pub;

  	ACFdetector()
  	  : it_(nh_)
  	{
  		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
  			                       &ACFdetector::detection, this);


      depth_points_sub = n.subscribe("/camera/depth_registered/points", 1, &ACFdetector::depthCallBack, this);
      classify_flag_sub = n.subscribe("/vision/trigger_classification", 1, &ACFdetector::flagCallBack, this);
      obj_detected_pub = n.advertise<acfDetector::detectedObj>("/vision/object", 1);
      obj_state_pub = n.advertise<robo7_msgs::detectedState>("/vision/state", 1);

      std::string acfModel;
      n.param<string>("/acf_detector/acfModel", acfModel, "/home/ras17/catkin_ws/src/robo7/vision/detection/acfDetector/model/model_color_mag.cpb");
      detector = std::make_shared<acf::Detector>(acfModel);

      if (detector.get() && detector->good()) {
        detector->setDoNonMaximaSuppression(true);
      }

      n.param<double>("/acf_detector/scoreThre", scoreThre, 50);
      n.param<double>("/acf_detector/sizeMin", sizeMin, 50);
      n.param<double>("/acf_detector/sizeAccept", sizeAccept, 110);
      n.param<double>("/acf_detector/boundaryForward", boundaryForward, 60);
      n.param<double>("/acf_detector/boundaryThre", boundaryThre, 30);
      n.param<bool>("/acf_detector/showFlag", showFlag, true);
      n.param<bool>("/acf_detector/svmFlag", svmFlag, true);

      if (showFlag == true)
        namedWindow("Detected image");

      // namedWindow("bbx");
      // namedWindow("Cropped object image");

      // record a Video
      // video.open("/home/ras17/detected_result.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, Size(640, 480));
      // if (video.isOpened())
      //   ROS_INFO("Initialize successfully!");
      // else
      //   ROS_INFO("Initializion fail!");

    }

  	~ACFdetector()
  	{
  		destroyAllWindows();
  	}

    void flagCallBack(const std_msgs::Bool &msg)
    {
      classify_flag = msg.data;
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

      float X = 0.0, Y = 0.0, Z = 0.0;

      memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

      // ROS_INFO("xx: %f", X); ROS_INFO("yy: %f", Y); ROS_INFO("zz: %f", Z);

      geometry_msgs::Point p;
      p.x = X; p.y = Y; p.z = Z;

      return p;
    }

    cv::Rect enlargeBBX(cv::Rect o, int cols, int rows, float scale)
    {
      cv::Rect large_bbx;

      if(o.width >= rows/scale)
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
        int x_tl = o.x - o.width * (scale-1)/2;
        int y_tl = o.y - o.height * (scale-1)/ 2;
        int x_br = o.x + o.width * (1+(scale-1)/2);
        int y_br = o.y + o.height * (1+(scale-1)/2);

        if (x_tl >= 0)
          large_bbx.x = x_tl;
        else
          large_bbx.x = 0;
        if (y_tl >= 0)
          large_bbx.y = y_tl;
        else
          large_bbx.y = 0;
        if (x_br >= cols)
          large_bbx.x = cols - scale * o.width;
        if (y_br >= rows)
          large_bbx.y = rows - scale * o.height;

        large_bbx.width = scale * o.width;
        large_bbx.height = scale * o.height;
      }
      return large_bbx;
    }

    cv::Mat get_color_histogram(cv::Mat img){
        cv::Mat img_hsv;
        cvtColor(img, img_hsv, CV_BGR2HSV);
        vector<Mat> hsv_planes;
        split(img_hsv, hsv_planes);

        Mat hist_h_mat, hist_s_mat, hist_v_mat;
        int hSize = 181, svSize = 256;
        float hrange[] = {0, 180}, svrange[] = {0, 256};
        const float* hRange = {hrange};
        const float* svRange = {svrange};

        calcHist(&hsv_planes[0], 1, 0, Mat(), hist_h_mat, 1, &hSize, &hRange, true, false);
        hist_h_mat.convertTo(hist_h_mat, CV_32FC1);
        hist_h_mat = hist_h_mat / width / height;

        calcHist(&hsv_planes[1], 1, 0, Mat(), hist_s_mat, 1, &svSize, &svRange, true, false);
        hist_s_mat.convertTo(hist_s_mat, CV_32FC1);
        hist_s_mat = hist_s_mat / width / height;

        calcHist(&hsv_planes[2], 1, 0, Mat(), hist_v_mat, 1, &svSize, &svRange, true, false);
        hist_v_mat.convertTo(hist_v_mat, CV_32FC1);
        hist_v_mat = hist_v_mat / width / height;

        cv::Mat hist;
        vconcat(hist_h_mat, hist_s_mat, hist);
        vconcat(hist, hist_v_mat, hist);
        transpose(hist, hist);

        return hist;
    }

    void applyDetector(AcfPtr detector, cv::Mat origImg)
    {
      std::vector<double> scores;
      std::vector<cv::Rect> objects;
      string color[7] = {"yellow", "green", "orange", "red", "blue", "purple", "unknown"};
      std::vector<string> colorVec(color, color+7);

      cv::Mat imageRGB;
      cv::cvtColor(origImg, imageRGB, cv::COLOR_BGR2RGB);
      (*detector)(imageRGB, objects, &scores);

      int ind = 0;
      std::stringstream ss;

      robo7_msgs::detectedState state_msg;

      for (const auto& o : objects)
      {
        // Threshold of scores
        if (scores[ind] < scoreThre){
          // ROS_INFO("skip the bbx with %f", scores[ind]);
          ind++;
          continue;
        }

        ROS_INFO("size of bbx:%d", o.width);
        if (o.width <= sizeMin){
          ind++;
          ROS_INFO("too small bbx!");
          continue;
        }

        if (o.width > sizeMin && o.width < sizeAccept){
          ind++;

          if (o.x < boundaryForward){
            ROS_INFO("should move forward left!");
            state_msg.state.push_back(4);
          }
          if ((o.x+o.width) > (origImg.cols - boundaryForward)){
            ROS_INFO("should move forward right!");
            state_msg.state.push_back(5);
          }
          else{
            ROS_INFO("should move forward!");
            state_msg.state.push_back(6);
          }
          continue;
        }

        // ROS_INFO("left x: %d", o.x);
        // ROS_INFO("right x: %d", o.x + o.width);
        if (o.x < boundaryThre){
          ind++;
          state_msg.state.push_back(1);
          continue;
          ROS_INFO("too left!");
        }

        if ((o.x + o.width) > (origImg.cols - boundaryThre)){
          ind++;
          state_msg.state.push_back(2);
          continue;
          ROS_INFO("too right!");
        }

        acfDetector::detectedObj object_pub;

        // make sure the bbx don't out of the image
        if (o.x >= 0 && o.y >= 0 && (o.x + o.width) <= origImg.cols && (o.y + o.height) <= origImg.rows)
        {
          state_msg.state.push_back(3);

          Point center = (o.tl()+o.br())/2;
          // draw bbbx
          if (showFlag == true)
          {
            cv::rectangle(resultImg, o, { 0, 255, 0 }, 2, 8);
            circle(resultImg, center, 2, Scalar(0,0,255), 2, 8, 0);
          }

          int response = 6;
          if (svmFlag == true)
          {
            // apply SVM color classifier
            cv::Mat bbx_img;
            std::vector<float> hist;
            bbx_img = origImg(o);
            cv::resize(bbx_img, bbx_img, cv::Size(width, height));
            hist = ACFdetector::get_color_histogram(bbx_img);
            int response = svm->predict(hist);
            // cout << colorVec[response] << endl;
            // imshow("bbx", bbx_img);
            // waitKey(5);
          }


          if (showFlag == true)
          {
            // put the score and color on the bbx
            // ROS_INFO("score: %f", scores[ind]);
            ss << scores[ind];
            cv::putText(resultImg, ss.str().substr(0, 4) + ", " + colorVec[response], cv::Point(o.x, o.y), CV_FONT_HERSHEY_SIMPLEX, 0.8, {255, 255, 255});
          }

          ind++;

          if (classify_flag == true){
            // Crop the image with enlarged bbx
            cv::Rect large_o = ACFdetector::enlargeBBX(o, origImg.cols, origImg.rows, 1.4);
            cv::Mat obj_img;
            obj_img = origImg(large_o);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", obj_img).toImageMsg();

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
            }

            // Publish msg
            // ROS_INFO("pos.x: %f", pos.x);
            object_pub.pos = pos;
            std_msgs::String color;
            color.data = colorVec[response];
            object_pub.color = color;
            // object_pub.objClass = 1;

            obj_detected_pub.publish(object_pub);
          }

        }
      }

      obj_state_pub.publish(state_msg);
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
        ACFdetector::applyDetector(detector, origImg);


        if (showFlag == true)
        {
          imshow("Detected image", resultImg);
          cv::waitKey(5);
        }

        // video
        // video.write(resultImg);

      }

  	}

private:
	cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::PointCloud2 pCloud_cam;

  AcfPtr detector;

  cv::Mat resultImg;
  double scoreThre;
  double sizeAccept;
  double sizeMin;
  double boundaryThre;
  double boundaryForward;
  bool showFlag;
  bool svmFlag;
  bool classify_flag;

  // cv::VideoWriter video;


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "acf_detector");

	ACFdetector node;

  ros::Rate loop_rate(2);

	ros::spin();

	return 0;

}
