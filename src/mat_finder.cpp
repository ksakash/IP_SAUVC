// Copyright 2017 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <IP_SAUVC/matConfig.h>
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
// virtual int alpha;
cv::Mat frame, newframe, dst1, image_clahe, src;
std_msgs::Float64MultiArray array;

void callback(IP_SAUVC::matConfig &config, uint32_t level)
{
  m1min = config.m1min_param;
  m1max = config.m1max_param;
  m2min = config.m2min_param;
  m2max = config.m2max_param;
  m3min = config.m3min_param;
  m3max = config.m3max_param;
  ROS_INFO("Mat_Reconfigure Request:New params : %d %d %d %d %d %d", m1min, m1max, m1min, m2max, m3min, m3max);
}

void matFinderListener(std_msgs::Bool msg){
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

int get_largest_contour_index(std::vector<std::vector<cv::Point> > contours){

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

cv::Point2f get_contour_center(std::vector<cv::Point> contour){

  std::vector<std::vector<cv::Point> > hull(1);
  cv::convexHull(cv::Mat(contour), hull[0], false);

  cv::Moments mu;
  mu = cv::moments(hull[0], false);
  cv::Point2f center_of_mass;

  center_of_mass = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

  return center_of_mass;

}

cv::Mat color_correction(cv::Mat &src, int parameter){ // same for all the tasks

  std::vector<cv::Mat> lab_planes(3);
  cv::Mat dst, lab_image;

  cv::cvtColor(src, lab_image, CV_BGR2Lab);

  // Extract the L channel
  cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

  // apply the CLAHE algorithm to the L channel
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(parameter);

  clahe->apply(lab_planes[0], dst);

  // Merge the the color planes back into an Lab image
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);

  // convert back to RGB
  cv::Mat image_clahe;
  cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

  return image_clahe;

}

void denoise(cv::Mat &src, int i){ // may be needed in a task
  cv::Mat dstx;

  for (int j = 0; j < i; j++){
    bilateralFilter(src, dstx, 6, 8, 8);
    bilateralFilter(dstx, src, 6, 8, 8);
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "mat_finder");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/mat/finder", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("mat_finder_switch", 1000, &matFinderListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  // frame = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR );
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw",1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<IP_SAUVC::matConfig> server;
  dynamic_reconfigure::Server<IP_SAUVC::matConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cv::Mat balanced_image, dst, thresholded;
  cv::Scalar color = cv::Scalar( 0, 0, 255 );

  while (ros::ok()){

    loop_rate.sleep();

    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    frame.copyTo(src);
    dst1 = balance_white(frame, 0.05);
    image_clahe = color_correction(frame, 4);
    denoise(image_clahe, 2);
    cv::Mat drawing(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));
    balanced_image = balance_white(image_clahe, 0.05);
    denoise(balanced_image, 2);
    sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst1).toImageMsg();
    pub3.publish(msg3);

    cv::Scalar hsv_min = cv::Scalar(m1min, m2min, m3min, 0);
    cv::Scalar hsv_max = cv::Scalar(m1max, m2max, m3max, 0);

    cv::inRange(dst1, hsv_min, hsv_max, thresholded);

    // cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();
    pub1.publish(msg1);

    if (1){ // IP here
      // if half of the screen is green then we are inside the Mat
      std::vector<std::vector<cv::Point> > contours;
      findContours(thresholded, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image

      if (!contours.empty()){

        int largest_contour_index = get_largest_contour_index(contours);
        cv::RotatedRect minRect;
        double area = 0;
        double angle = 0;
        minRect = cv::minAreaRect(cv::Mat(contours[largest_contour_index]));

        cv::Point2f rect_points[4];
        minRect.points(rect_points);

        for( int j = 0; j < 4; j++ ){
          cv::line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
          cv::line( src, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        }

        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        pub2.publish(msg2);

        double a = sqrt(distance(rect_points[0], rect_points[1]));
        double b = sqrt(distance(rect_points[1], rect_points[2]));

        if (a > b){
          angle = (rect_points[1].y - rect_points[0].y)/(rect_points[1].x - rect_points[0].x);
        }
        else if ( b > a){
          angle = (rect_points[2].y - rect_points[1].y)/(rect_points[2].x - rect_points[1].x);
        }

        cv::Point2f center_of_mass = get_contour_center(contours[largest_contour_index]);
        area = contourArea(contours[largest_contour_index], false);
        array.data.push_back(center_of_mass.x);
        array.data.push_back(center_of_mass.y);
        array.data.push_back(area);
        array.data.push_back(angle);

        pub.publish(array);

        ros::spinOnce();
        continue;

      }

      else {

        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);
        array.data.push_back(0);

        pub.publish(array);

        ros::spinOnce();
        continue;

      }
    }
  }
}
