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
#include <test_pkg/gateConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include <math.h>
cv::RNG rng(12345);

#define SHELLSCRIPT_DUMP "\
#/bin/bash \n\
echo -e \"parameters dumped!!\" \n\
rosparam dump ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml /buoy_detection\
"
#define SHELLSCRIPT_LOAD "\
#/bin/bash \n\
echo -e \"parameters loaded!!\" \n\
rosparam load ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml /buoy_detection\
"
typedef std::vector<std::vector<cv::Point2f> > contour_array;

cv::Mat frame, newframe, balanced_image, dst1;
std::vector<cv::Mat> thresholded(3);
contour_array contour0;
contour_array contour1;
contour_array contour2;

int red_min[3], red_max[3], green_min[3], green_max[3], blue_min[3], blue_max[3]; // to store the color RGB values of all the rods
int flag = 0;
bool threshold = false;
bool save = false;
int count = 0;
bool task_gate_done;
bool gate_found;

void callback(test_pkg::gateConfig &config, uint32_t level){

  flag = config.flag_param;
  threshold = config.threshold_param;

  if(!threshold){

    // update the values of the trackbars used in thresholding the image

    if (flag == 0){
      config.t1min_param = red_min[0];
      config.t1max_param = red_max[0];
      config.t2min_param = blue_min[0];
      config.t2max_param = blue_max[0];
      config.t3min_param = green_min[0];
      config.t3max_param = green_max[0];
    }
    else if (flag == 1){
      config.t1min_param = red_min[1];
      config.t1max_param = red_max[1];
      config.t2min_param = blue_min[1];
      config.t2max_param = blue_max[1];
      config.t3min_param = green_min[1];
      config.t3max_param = green_max[1];
    }
    else if (flag == 2){
      config.t1min_param = red_min[2];
      config.t1max_param = red_max[2];
      config.t2min_param = blue_min[2];
      config.t2max_param = blue_max[2];
      config.t3min_param = green_min[2];
      config.t3max_param = green_max[2];
    }

  }

  // update the values in the red, green, blue matrix

  if (flag == 0){

    red_min[0] = config.t1min_param;
    red_max[0] = config.t1max_param;
    blue_min[0] = config.t2min_param;
    blue_max[0] = config.t2max_param;
    green_min[0] = config.t3min_param;
    green_max[0] = config.t3max_param;

  }

  else if (flag == 1){

    red_min[1] = config.t1min_param;
    red_max[1] = config.t1max_param;
    blue_min[1] = config.t2min_param;
    blue_max[1] = config.t2max_param;
    green_min[1] = config.t3min_param;
    green_max[1] = config.t3max_param;

  }

  else if (flag == 2){

    red_min[1] = config.t1min_param;
    red_max[1] = config.t1max_param;
    blue_min[1] = config.t2min_param;
    blue_max[1] = config.t2max_param;
    green_min[1] = config.t3min_param;
    green_max[1] = config.t3max_param;

  }

  if (!count){

    config.save_param = false; // just for making the radio button for save parameter false at the start of thresholding
    count++;

  }

  save = config.save_param;

  // for informing about the changing parameters
  ROS_INFO("Gate_Reconfigure Request:Red_Pole_params : %d %d %d %d %d %d %d %d %d", red_min[0], red_max[0], blue_min[0], blue_max[0], green_min[0], green_max[0], save, flag, threshold);
  ROS_INFO("Gate_Reconfigure Request:Green_Pole_params : %d %d %d %d %d %d %d %d %d", red_min[1], red_max[1], blue_min[1], blue_max[1], green_min[1], green_max[1]);

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
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}

bool isGateDectected(contour_array contour0, contour_array contour1, contour_array contour2){

  return ((!contour0.empty())||(!contour1.empty())||(!contour2.empty()));

}

int get_largest_contour_index(contour_array contours){ // to get the leargest contour index in a group of contours

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

cv::Point2f get_center_of_contour(contour_array contours){ // to get the center of the contour

  int largest_contour_index = get_largest_contour_index(contours);
  std::vector<std::vector<cv::Point> > hull(1);
  cv::convexHull(cv::Mat(contours[largest_contour_index]), hull[0], false);

  cv::Moments mu;
  mu = cv::moments(hull[0], false);
  cv::Point2f center_of_mass;

  center_of_mass = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

  return center_of_mass;

}

void draw_min_fit_rectangles(cv::Mat &src, cv::Mat &drawing, contour_array contour0, contour_array contour1, contour_array contour2)
{
  // to draw the detected contours on the blank image
  if (!contour0.empty()){
    std::vector<cv::Vec4i> hierarchy;
    int largest_contour_index = get_largest_contour_index(contour0);
    cv::RotatedRect minRect;
    minRect = cv::minAreaRect(cv::Mat(contour0[largest_contour_index]));

    cv::Point2f rect_points[4]; minRect.points( rect_points );
    cv::Scalar color = cv::Scalar( 0, 0, 255 );
    for( int j = 0; j < 4; j++ ){
       cv::line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
       cv::line( src, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }
    cv::drawContours( drawing, contour0, largest_contour_index, color, 1, 8, hierarchy);
  }

  if (!contour1.empty()){
    std::vector<cv::Vec4i> hierarchy;
    int largest_contour_index = get_largest_contour_index(contour1);
    cv::RotatedRect minRect;
    minRect = cv::minAreaRect(cv::Mat(contour0[largest_contour_index]));

    cv::Point2f rect_points[4]; minRect.points( rect_points );
    cv::Scalar color = cv::Scalar( 0, 255, 0 );
    for( int j = 0; j < 4; j++ ){
       cv::line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
       cv::line( src, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }
    cv::drawContours( drawing, contour1, largest_contour_index, color, 1, 8, hierarchy);
  }

  if (!contour2.empty()){
    std::vector<cv::Vec4i> hierarchy;
    int largest_contour_index = get_largest_contour_index(contour2);
    cv::RotatedRect minRect;
    minRect = cv::minAreaRect(cv::Mat(contour2[largest_contour_index]));

    cv::Point2f rect_points[4]; minRect.points( rect_points );
    cv::Scalar color = cv::Scalar( 255, 255, 255 );
    for( int j = 0; j < 4; j++ ){
       cv::line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
       cv::line( src, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }

    cv::drawContours( drawing, contour2, largest_contour_index, color, 1, 8, hierarchy);
  }
   return;
}

cv::Point2f get_gate_center(contour_array contour0, contour_array contour1, contour_array contour2){

  cv::Point2f red_rod_center;
  cv::Point2f green_rod_center;
  cv::Point2f black_rod_center;
  cv::Point2f gate_center;

  if (!contour0.empty())
    red_rod_center = get_center_of_contour(contour0);

  if (!contour1.empty())
    green_rod_center = get_center_of_contour(contour1);

  if (!contour2.empty())
    black_rod_center = get_center_of_contour(contour2);

  if (contour0.empty())
    red_rod_center = green_rod_center;

  else if (contour1.empty())
    green_rod_center = red_rod_center;

  if (contour2.empty()){
    gate_center.x = (red_rod_center.x + green_rod_center.x)/2;
    gate_center.y = (red_rod_center.y + green_rod_center.y)/2;
  }

  else if (!contour2.empty()){
    gate_center.x = black_rod_center.x;
    gate_center.y = (red_rod_center.y + green_rod_center.y)/2;
  }

  return gate_center;

}

double distance(cv::Point2f a, cv::Point2f b){

  return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);

}

void draw_gate(cv::Mat &src, contour_array contour0, contour_array contour1, contour_array contour2, cv::Point2f gate_center){

  if (contour2.empty()){
      return;
  }

  else {

    std::vector<cv::RotatedRect> minRect(2);
    int index0 = get_largest_contour_index(contour2);
    int index1;

    minRect[0] = cv::minAreaRect( cv::Mat(contour2[index0]) );

    if (!contour0.empty()){
      index1 = get_largest_contour_index(contour0);
      minRect[1] = cv::minAreaRect( cv::Mat(contour0[index1]) );
    }

    if (!contour1.empty()){
      index1 = get_largest_contour_index(contour1);
      minRect[1] = cv::minAreaRect( cv::Mat(contour1[index1]) );
    }

    cv::Point2f rect_points[4];
    minRect[0].points( rect_points );
    double L, B;
    double a = distance(rect_points[0], rect_points[1]);
    double b = distance(rect_points[1], rect_points[2]);

    if (a > b){
      L = a;
    }
    else {
      L = b;
    }

    minRect[1].points( rect_points );

    a = distance(rect_points[0], rect_points[1]);
    b = distance(rect_points[1], rect_points[2]);

    if ( a > b){
      B = a;
    }
    else {
      B = b;
    }

    cv::Point2f X, Y;

    X = cv::Point2f(gate_center.x + sqrt(L)/2, gate_center.y + sqrt(B)/2);
    Y = cv::Point2f(gate_center.x - sqrt(L)/2, gate_center.y - sqrt(B)/2);

    cv::rectangle(src, X, Y, cv::Scalar(255, 255, 255), 2, 8, 0);

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

int main(int argc, char **argv){

  ros::init(argc, argv, "gate_detection");
  int length, breadth, step, channels;
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/varun/ip/gate", 1000);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  std_msgs::Float64MultiArray array;

  /// to load all the parameters used in the node from the yaml file on the parameter server
  system(SHELLSCRIPT_LOAD);

  /// to get those parameters from the parameter srver and set it in the variables declared here

  nh.getParam("gate_detection/r1min", red_min[0]);
  nh.getParam("gate_detection/r1max", red_max[0]);
  nh.getParam("gate_detection/r2min", blue_min[0]);
  nh.getParam("gate_detection/r2max", blue_max[0]);
  nh.getParam("gate_detection/r3min", green_min[0]);
  nh.getParam("gate_detection/r3max", green_max[0]);

  nh.getParam("gate_detection/g1max", red_min[1]);
  nh.getParam("gate_detection/g1min", red_max[1]);
  nh.getParam("gate_detection/g2max", blue_min[1]);
  nh.getParam("gate_detection/g2min", blue_max[1]);
  nh.getParam("gate_detection/g3max", green_min[1]);
  nh.getParam("gate_detection/g3min", green_max[1]);

  nh.getParam("gate_detection/b1max", red_min[2]);
  nh.getParam("gate_detection/b1min", red_max[2]);
  nh.getParam("gate_detection/b2max", blue_min[2]);
  nh.getParam("gate_detection/b2min", blue_max[2]);
  nh.getParam("gate_detection/b3max", green_min[2]);
  nh.getParam("gate_detection/b3min", green_max[2]);

  dynamic_reconfigure::Server<test_pkg::gateConfig> server;
  dynamic_reconfigure::Server<test_pkg::gateConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cv::Scalar color = cv::Scalar(255, 255, 255);
  cv::Point2f gate_center;
  int net_x_cord;
  int net_y_cord;

  while(ros::ok()){

    if (!task_gate_done){

      loop_rate.sleep();

      if (save == true){

        nh.setParam("gate_detection/r1min", red_min[0]); // r and 0 for red rod
        nh.setParam("gate_detection/r1max", red_max[0]);
        nh.setParam("gate_detection/r2min", blue_min[0]);
        nh.setParam("gate_detection/r2max", blue_max[0]);
        nh.setParam("gate_detection/r3min", green_min[0]);
        nh.setParam("gate_detection/r3max", green_max[0]);

        nh.setParam("gate_detection/g1min", red_min[1]); // g and 1 for green rod
        nh.setParam("gate_detection/g1max", red_max[1]);
        nh.setParam("gate_detection/g2min", blue_min[1]);
        nh.setParam("gate_detection/g2max", blue_max[1]);
        nh.setParam("gate_detection/g3min", green_min[1]);
        nh.setParam("gate_detection/g3max", green_max[1]);

        nh.setParam("gate_detection/b1min", red_min[2]); // b and 2 for black rod
        nh.setParam("gate_detection/b1max", red_max[2]);
        nh.setParam("gate_detection/b2min", blue_min[2]);
        nh.setParam("gate_detection/b2max", blue_max[2]);
        nh.setParam("gate_detection/b3min", green_min[2]);
        nh.setParam("gate_detection/b3max", green_max[2]);

        /// dump the values in the yaml file
        save = false;
        system(SHELLSCRIPT_DUMP); // all the parameters saved in the yaml file

      }

      if (frame.empty())
      {
        ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
        ros::spinOnce();
        continue;
      }

      /// all the image proecssing till the threshold

      frame.copyTo(balanced_image);
      cv::Mat drawing(frame.rows, frame.cols, CV_8UC1, cv::Scalar::all(0));
      balance_white(balanced_image);
      bilateralFilter(balanced_image, dst1, 4, 8, 8);

      cv::Scalar red_rod_min = cv::Scalar(blue_min[0], green_min[0], red_min[0], 0);
      cv::Scalar red_rod_max = cv::Scalar(blue_max[0], green_max[0], red_max[0], 0);

      cv::Scalar green_rod_min = cv::Scalar(blue_min[1], green_min[1], red_min[1], 0);
      cv::Scalar green_rod_max = cv::Scalar(blue_max[1], green_max[1], red_max[1], 0);

      cv::Scalar black_rod_min = cv::Scalar(blue_min[2], green_min[2], red_min[2], 0);
      cv::Scalar black_rod_max = cv::Scalar(blue_max[2], green_max[2], red_max[2], 0);

      /// thresholding all the colors according to their thresholding values
      cv::inRange(dst1, red_rod_min, red_rod_max, thresholded[0]);
      cv::inRange(dst1, green_rod_min, green_rod_max, thresholded[1]);
      cv::inRange(dst1, black_rod_min, black_rod_max, thresholded[2]);

      for (int i = 0; i < 3; i++)
      {
        cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
      }

      findContours(thresholded[0], contour0, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      findContours(thresholded[1], contour1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      findContours(thresholded[2], contour2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

      // int largest_contour_index0 = get_largest_contour_index(contour0);
      // int largest_contour_index1 = get_largest_contour_index(contour1);
      // int largest_contour_index2 = get_largest_contour_index(contour2);

      if (!gate_found){
        gate_found = isGateDectected(contour0, contour1, contour2);
        continue;
      }

      draw_min_fit_rectangles(frame, drawing, contour0, contour1, contour2);

      if (gate_found){

        // if contour is not empty
        if (isGateDectected(contour0, contour1, contour2)){ // if there is even one contour

          gate_center = get_gate_center(contour0, contour1, contour2);
          draw_gate(frame, contour0, contour1, contour2, gate_center);

          net_x_cord = gate_center.x - 320; // net_x_cord & net_y_cord for the coordinate w.r.t screen center
          net_y_cord = 240 - gate_center.y;

          if (net_x_cord < -270)
          {
            array.data.push_back(-2);  // move left
            array.data.push_back(-2);
            pub.publish(array);
          }
          else if (net_x_cord > 270)
          {
            array.data.push_back(-1);  // move right
            array.data.push_back(-1);
            pub.publish(array);
          }
          else if (net_y_cord > 200)
          {
            array.data.push_back(-3); // move up
            array.data.push_back(-3);
            pub.publish(array);
          }
          else if (net_y_cord < -200)
          {
            array.data.push_back(-4);  // move down
            array.data.push_back(-4);
            pub.publish(array);
          }
          else
          {
            array.data.push_back(net_x_cord);
            array.data.push_back(net_y_cord);
            pub.publish(array);
          }

          ros::spinOnce();

        }
        // if contour is empty
        else {

          if (net_x_cord < -270)
          {
            array.data.push_back(-2);  // move left
            array.data.push_back(-2);
            pub.publish(array);
          }
          else if (net_x_cord > 270)
          {
            array.data.push_back(-1);  // move right
            array.data.push_back(-1);
            pub.publish(array);
          }
          else if (net_y_cord > 200)
          {
            array.data.push_back(-3);  // move up
            array.data.push_back(-3);
            pub.publish(array);
          }
          else if (net_y_cord < -200)
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

    } // if ends

  } // while ends

}
