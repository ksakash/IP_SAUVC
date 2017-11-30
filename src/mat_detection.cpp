// Copyright 2017 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <test_pkg/matConfig.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float32MultiArray.h"
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include <math.h>

bool IP = false;
int m1min, m1max, m2min, m2max, m3min, m3max;

cv::Mat frame, newframe;

void callback(test_pkg::matConfig &config, uint32_t level)
{
  m1min = config.m1min_param;
  m1max = config.m1max_param;
  m1min = config.m1min_param;
  m2max = config.m2max_param;
  m3min = config.m3min_param;
  m3max = config.m3max_param;
  ROS_INFO("Mat_Reconfigure Request:New params : %d %d %d %d %d %d", m1min, m1max, m1min, m2max, m3min, m3max);
}

void matDetectionListener(std_msgs::Bool msg){
    IP = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try{
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    newframe.copyTo(frame);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}

cv::Mat balance_white(cv::Mat src, float parameter){ // same for all the tasks

  float discard_ratio = parameter;
  cv::Mat mat = src;

  int hists[3][256];
  memset(hists, 0, 3*256*sizeof(int));

  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols*mat.rows;
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 255; ++j) {
      hists[i][j + 1] += hists[i][j];
    }
    vmin[i] = 0;
    vmax[i] = 255;
    while (hists[i][vmin[i]] < discard_ratio * total)
      vmin[i] += 1;
    while (hists[i][vmax[i]] > (1 - discard_ratio) * total)
      vmax[i] -= 1;
    if (vmax[i] < 255 - 1)
      vmax[i] += 1;
  }

  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        int val = ptr[x * 3 + j];
        if (val < vmin[j])
          val = vmin[j];
        if (val > vmax[j])
          val = vmax[j];
        ptr[x * 3 + j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
      }
    }
  }

  return mat;
}

int get_largest_contour_index(std::vector<std::vector<cv::Point2f> > contours){

  int largest_contour_index = 0;
  double largest_area = 0;

  for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
  {
    double a = contourArea(contours[i], false);  //  Find the area of contour
    if (a > largest_area)
    {
      largest_area = a;
      largest_contour_index = i;  // Store the index of largest contour
    }
  }

  return largest_contour_index;

}

double distance(cv::Point2f a, cv::Point2f b){
  return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "mat_detector");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Int32>("/varun/ip/mat", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("mat_detection_switch", 1000, &basketDetectedListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw",1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<test_pkg::ballConfig> server;
  dynamic_reconfigure::Server<test_pkg::ballConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cv::Mat balanced_image, dst, thresholded;

  while (ros::ok()){

    std_msgs::Int32 mat_status;
    loop_rate.sleep();

    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    frame.copyTo(balanced_image);
    balance_white(balanced_image);
    bilateralFilter(balanced_image, dst, 4, 8, 8);

    cv::Scalar hsv_min = cv::Scalar(m1min, m2min, m3min, 0);
    cv::Scalar hsv_max = cv::Scalar(m1max, m2max, m3max, 0);

    cv::inRange(dst, hsv_min, hsv_max, thresholded);

    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    if (IP){
      // if half of the screen is green then we are inside the Mat
      std::vector<std::vector<cv::Point> > contours;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image

      if (!contours.empty()){
        int largest_contour_index = get_largest_contour_index(contours);
        cv::RotatedRect minRect;
        minRect = cv::minAreaRect(cv::Mat(contours[leargest_contour_index]));
        cv::Point2f rect_points[4];
        minRect.points(rect_points);

        double a = sqrt(distance(rect_points[0], rect_points[1]));
        double b = sqrt(distance(rect_points[1], rect_points[2]));

        double area = a*b;
        double percentage = area/(640*480);

        if (percentage >= 0.5){
          mat_status = 1;
        }
        else if (percentage <= 0.05){
          mat_status = -1;
        }
        else {
          mat_status = 0;
        }

        pub.publish(mat_status);
        ros::spinOnce();
      }

      if (contours.empty()){

        continue;

      }
    }

  }


}
