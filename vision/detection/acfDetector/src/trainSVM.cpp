//
// Created by jtao on 2018-11-12.
//
#include <ros/ros.h>
#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include <opencv2/ml.hpp>
//#include <glob.h>

using namespace cv;
using namespace cv::ml;
using namespace std;

int width = 80, height = 80;

void loadData(const String dataDir, vector<cv::String> &addrs, vector<int> &labels) {

    glob(dataDir, addrs);

    // shuffle the dataset
    std::shuffle(std::begin(addrs), std::end(addrs), std::default_random_engine());

    for (int i=0; i<addrs.size(); i++){
        if (addrs[i].find("Y_ball") != string::npos)
            labels.push_back(0);
        else if (addrs[i].find("Y_cube") != string::npos)
            labels.push_back(0);
        else if (addrs[i].find("G_cube") != string::npos)
            labels.push_back(1);
        else if (addrs[i].find("G_cylinder") != string::npos)
            labels.push_back(1);
        else if (addrs[i].find("G_hollow") != string::npos)
            labels.push_back(1);
        else if (addrs[i].find("O_cross") != string::npos)
            labels.push_back(2);
        else if (addrs[i].find("O_star") != string::npos)
            labels.push_back(2);
        else if (addrs[i].find("R_cylinder") != string::npos)
            labels.push_back(3);
        else if (addrs[i].find("R_hollow") != string::npos)
            labels.push_back(3);
        else if (addrs[i].find("R_ball") != string::npos)
            labels.push_back(3);
        else if (addrs[i].find("B_cube") != string::npos)
            labels.push_back(4);
        else if (addrs[i].find("B_triangle") != string::npos)
            labels.push_back(4);
        else if (addrs[i].find("P_cross") != string::npos)
            labels.push_back(5);
        else if (addrs[i].find("P_star") != string::npos)
            labels.push_back(5);
        else
        {
            cout << i << endl;
            cout << addrs[i] << endl;
            cout << "Unexpected image name!" << endl;
        }

    }

}

void ConvertVectortoMatrix(vector<vector<float> > &trainHOG, vector<vector<float> > &testHOG, Mat &trainMat, Mat &testMat)
{

    int descriptor_size = int(trainHOG[0].size());

    for(int i = 0;i<trainHOG.size();i++){
        for(int j = 0;j<descriptor_size;j++){
            trainMat.at<float>(i,j) = trainHOG[i][j];
        }
    }
    for(int i = 0;i<testHOG.size();i++){
        for(int j = 0;j<descriptor_size;j++){
            testMat.at<float>(i,j) = testHOG[i][j];
        }
    }
}

std::vector<float> get_color_histogram(cv::Mat img){

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
    std::vector<float> hist_h;
    hist_h_mat.col(0).copyTo(hist_h);

    calcHist(&hsv_planes[1], 1, 0, Mat(), hist_s_mat, 1, &svSize, &svRange, true, false);
    hist_s_mat.convertTo(hist_s_mat, CV_32FC1);
    hist_s_mat = hist_s_mat / width / height;
    std::vector<float> hist_s;
    hist_s_mat.col(0).copyTo(hist_s);

    calcHist(&hsv_planes[2], 1, 0, Mat(), hist_v_mat, 1, &svSize, &svRange, true, false);
    hist_v_mat.convertTo(hist_v_mat, CV_32FC1);
    hist_v_mat = hist_v_mat / width / height;
    std::vector<float> hist_v;
    hist_v_mat.col(0).copyTo(hist_v);

    std::vector<float> hist;
    hist = hist_h;
    hist.insert(hist.end(), hist_s.begin(), hist_s.end());
    hist.insert(hist.end(), hist_v.begin(), hist_v.end());

    return hist;
}

void getSVMParams(SVM *svm)
{
    cout << "Kernel type     : " << svm->getKernelType() << endl;
    cout << "Type            : " << svm->getType() << endl;
    cout << "C               : " << svm->getC() << endl;
    cout << "Degree          : " << svm->getDegree() << endl;
    cout << "Nu              : " << svm->getNu() << endl;
    cout << "Gamma           : " << svm->getGamma() << endl;
}

void SVMtrain(Mat &trainMat,vector<int> &trainLabels, Mat &testResponse,Mat &testMat){

    Ptr<SVM> svm = SVM::create();
    svm->setGamma(0.50625);
    svm->setC(12.5);
    svm->setKernel(SVM::RBF);
    svm->setType(SVM::C_SVC);
    Ptr<TrainData> td = TrainData::create(trainMat, ROW_SAMPLE, trainLabels);
    svm->train(td);
    //svm->trainAuto(td);
    svm->save("SVMmodel_ros.xml");

    Ptr<SVM> svm_model = ml::SVM::load("SVMmodel_ros.xml");
    cout << "hahahahahahaha" << endl;
    svm_model->predict(testMat, testResponse);
    getSVMParams(svm);
}

void SVMevaluate(Mat &testResponse,float &count, float &accuracy,vector<int> &testLabels){

    for(int i=0;i<testResponse.rows;i++)
    {
        //cout << testResponse.at<float>(i,0) << " " << testLabels[i] << endl;
        if(testResponse.at<float>(i,0) == testLabels[i]){
            count = count + 1;
        }
    }
    accuracy = (count/testResponse.rows)*100;
}

void train(){
  cout << "Load the dataset!" << endl;
  const String dataDir = "/home/ras17/data/ColorClass/train/*.png";
  vector<cv::String> addrs;
  vector<int> labels;
  loadData(dataDir, addrs, labels);


  cout << "Calculate color histogram!" << endl;
  std::vector<std::vector<float> > color_hist;
  cv::Mat img;
  for (int i=0; i<addrs.size(); i++){
      img = imread(addrs[i], CV_LOAD_IMAGE_COLOR);
//        img = imread("/home/jtao/robo7_vision/data/ColorClass/train/P_star40.png");
//        imshow("img", img);
//        waitKey(5);
      if (img.empty())
      {
        cout << "empty image" << endl;
        continue;
      }

      std::vector<float> hist;
      hist = get_color_histogram(img);
      color_hist.push_back(hist);
  }

  cout << "Spliting data into training (90%) and test set (10%)..." << endl;
  int num = color_hist.size();
  cout << "Total num of dataset: " << num << endl;
  std::vector<std::vector<float> > color_hist_train, color_hist_test;
  std::vector<int> labels_train, labels_test;

  for (int i=0; i<=num*0.9; i++){
      color_hist_train.push_back(color_hist[i]);
      labels_train.push_back(labels[i]);
  }
  cout << "Num of training dataset: " << color_hist_train.size() << endl;
  for (int i=int(num*0.9); i<num; i++){
      color_hist_test.push_back(color_hist[i]);
      labels_test.push_back(labels[i]);
  }
  cout << "Num of testing dataset: " << color_hist_test.size() << endl;

  cout << "Start training ..." << endl;
  int hist_size = int(color_hist_train[0].size());
  Mat trainMat(int(color_hist_train.size()),hist_size,CV_32FC1);
  Mat testMat(int(color_hist_test.size()),hist_size,CV_32FC1);
  ConvertVectortoMatrix(color_hist_train,color_hist_test,trainMat,testMat);

  Mat testResponse;
  SVMtrain(trainMat,labels_train,testResponse,testMat);

  cout << "Evaluate the model ..." << endl;
  float count = 0;
  float accuracy = 0 ;
  SVMevaluate(testResponse,count,accuracy,labels_test);

  cout << "Accuracy        : " << accuracy << "%"<< endl;
}

int main(int argc, char** argv){

  ros::init(argc, argv, "trainSVM");
  train();

  ros::spin();
  return 0;
}
