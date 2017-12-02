// Copyright 2017 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <test_pkg/grabberConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

bool IP = false;
bool grabber_found = false;
int g1min, g1max, g2min, g2max, g3min, g3max;

cv::Mat frame, newframe;

void callback(test_pkg::grabberConfig &config, uint32_t level)
{
  g1min = config.g1min_param;
  g1max = config.g1max_param;
  g2min = config.g2min_param;
  g2max = config.g2max_param;
  g3min = config.g3min_param;
  g3max = config.g3max_param;
  ROS_INFO("grabber_Reconfigure Request : New parameters : %d %d %d %d %d %d ", g1min, g1max, g2min, g2max, g3min, g3max);
}

void grabberDetectionListener(std_msgs::Bool msg)
{
  IP = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    newframe.copyTo(frame);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void balance_white(cv::Mat mat)
{
  double discard_ratio = 0.05;
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

cv::Point2f get_contour_center(std::vector<cv::Point2f> contour){

  std::vector<std::vector<cv::Point> > hull(1);
  cv::convexHull(cv::Mat(contour), hull[0], false);

  cv::Moments mu;
  mu = cv::moments(hull[0], false);
  cv::Point2f center_of_mass;

  center_of_mass = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

  return center_of_mass;

}

double distance(cv::Point2f a, cv::Point2f b){
  return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}

bool isgrabberDetected(cv::Mat &src){
  /// criteria for deciding whether an image contains a grabber or not
  /// based on the area of the largest contour
  /// above a certain limit then the grabber is present
}

int main(int argc, char *argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on

  ros::init(argc, argv, "grabber_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/grabber", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("grabber_detection_switch", 1000, &grabberDetectionListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<test_pkg::grabberConfig> server;
  dynamic_reconfigure::Server<test_pkg::grabberConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // capture size -
  CvSize size = cvSize(width, height);

  cv::Mat balanced_image, thresholded, dst;
  cv::Point2f grabber_center, hitting_point, net_cord;

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
    // Get one frame
    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;

    frame.copyTo(balanced_image);
    balance_white(balanced_image);
    bilateralFilter(balanced_image, dst, 4, 8, 8);

    cv::Scalar hsv_min = cv::Scalar(g1min, g2min, g3min, 0);
    cv::Scalar hsv_max = cv::Scalar(g1max, g2max, g3max, 0);

    cv::inRange(dst, hsv_min, hsv_max, thresholded);

    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    if (IP)
    {
      // find contours
      std::vector<std::vector<cv::Point2f> > contours;
      cv::Mat thresholded_Mat = thresholded;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image
      int largest_contour_index = 0;

      cv::Mat Drawing(thresholded_Mat.rows, thresholded_Mat.cols, CV_8UC1, cv::Scalar::all(0));

      std::vector<cv::Vec4i> hierarchy;
      cv::Scalar color(255, 255, 255);

      sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();
      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image).toImageMsg();

      pub2.publish(msg2);
      pub3.publish(msg3);

      if (!grabber_found){
        grabber_found = isgrabberDetected(thresholded_Mat);
        continue;
      }

      if (grabber_found){

        // if contour is not empty
        if (!contours.empty()){ // if there is even one contour

          largest_contour_index = get_largest_contour_index(contours);
          // grabber_center = get_contour_center(contours[largest_contour_index]);
          hitting_point = get_hitting_point(frame, contours[largest_contour_index], 0.75);

          net_cord.x = hitting_point.x - 320; // net_x_cord & net_y_cord for the coordinate w.r.t screen center
          net_cord.y = 240 - hitting_point.y;

          if (net_cord.x < -270)
          {
            array.data.push_back(-2);  // move left
            array.data.push_back(-2);
            pub.publish(array);
          }
          else if (net_cord.x > 270)
          {
            array.data.push_back(-1);  // move right
            array.data.push_back(-1);
            pub.publish(array);
          }
          else if (net_cord.y > 200)
          {
            array.data.push_back(-3); // move up
            array.data.push_back(-3);
            pub.publish(array);
          }
          else if (net_cord.y < -200)
          {
            array.data.push_back(-4);  // move down
            array.data.push_back(-4);
            pub.publish(array);
          }
          else
          {
            array.data.push_back(net_cord.x);
            array.data.push_back(net_cord.y);
            pub.publish(array);
          }
          ros::spinOnce();
        }

        // if contour is empty

        else {

          if (net_cord.x < -270)
          {
            array.data.push_back(-2);  // move left
            array.data.push_back(-2);
            pub.publish(array);
          }
          else if (net_cord.x > 270)
          {
            array.data.push_back(-1);  // move right
            array.data.push_back(-1);
            pub.publish(array);
          }
          else if (net_cord.y > 200)
          {
            array.data.push_back(-3);  // move up
            array.data.push_back(-3);
            pub.publish(array);
          }
          else if (net_cord.y < -200)
          {
            array.data.push_back(-4);  // move down
            array.data.push_back(-4);
            pub.publish(array);
          }
          ros::spinOnce();
        }
      }
      else {
        // contour is always empty
        continue;
      }
    }
  }
  return 0;
}
