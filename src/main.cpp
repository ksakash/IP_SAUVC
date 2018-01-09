#include <iostream>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <vector>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

#include "color_constancy.hpp"

#include "laplacianBlend.hpp"

using namespace std;

/* main file */
int mode=1;
int fps=30;
cv::Mat image, newframe;

int main2(int argc, char *argv[]);

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  std::cout << "bro" << '\n';
  try
  {
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    newframe.copyTo(image);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
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


int main(int argc, char *argv[])
{


    if(mode==1)
    {
        main2( argc,argv);
    }

}
int k=0;


/**
 * @brief this function generates the exposedness weight map
 * @param input image 1
 * @param input image 2
 * @return output image is wweight map
 */
Mat generateEMasks(Mat s1,Mat s2){

    //mean and vairance of exposedness mask
    float _gMean = 0.5;
    float _gVariance = 0.25;

   Mat dst;

   Mat src;
   s1.copyTo(src);

   //Mat src2;
   //s2.copyTo(src2);
   //converting to floating point images and extracting illumination channel
   src.convertTo(src,CV_32FC3,1.0,0);
   //src2.convertTo(src2,CV_32FC3,1.0,0);

   cvtColor(src,src,CV_BGR2HSV_FULL);
   //cvtColor(src2,src2,CV_BGR2HSV_FULL);

   src.convertTo(src,CV_32FC3,1.0/255.0,0);
   //src2.convertTo(src2,CV_32FC3,1.0/255.0,0);


   src.copyTo(dst);
//   dst.convertTo(dst,CV_32FC3,1,0);

   std::vector<cv::Mat_<float> > ichan,ichan1;
   std::vector<cv::Mat_<float> > dchan,dchan1;

    cv::split(src,ichan);
    //cv::split(src2,ichan1);

    cv::split(dst,dchan);
    //cv::split(dst,dchan1);




    int idx=2;



    cv::add(ichan[idx],-_gMean,dchan[idx]);



    multiply(dchan[idx],dchan[idx],dchan[idx], 1.0/(2*_gVariance*_gVariance));
    cv::exp(-dchan[idx], dchan[idx]);



    idx=2;
    //cv::add(ichan1[idx],-_gMean,dchan1[idx]);
    //multiply(dchan1[idx],dchan1[idx],dchan1[idx], 1.0/(2*_gVariance*_gVariance));
    //cv::exp(-dchan1[idx], dchan1[idx]);



    cvtColor(src,src,CV_HSV2BGR_FULL);
    //cvtColor(src2,src2,CV_HSV2BGR_FULL);


    dchan[idx].convertTo(dchan[idx],CV_32FC1,1.0,0);


     return dchan[idx];

}


/**
 * @brief function to compute global contrast
 * @param input image is BGR image
 * @return ouput image is a scalar weight map
 */
Mat localContrast(Mat s)
{
    Mat image;
    s.copyTo(image);

    Mat dst;
    cvtColor(image,image,CV_BGR2YCrCb);
    image.convertTo(image,CV_32FC3,1.0/255.0,0);
    std::vector<cv::Mat_<float> > d;
    cv::split(image,d);

    Mat tmp;
    //tmp.create(image.row,image.cols,CV_8UC1);
    d[0].copyTo(tmp);
    tmp.setTo(cv::Scalar::all(0));
    cv::Laplacian(d[0],d[0],image.depth(),3,1,0);
    cv::absdiff(d[0],tmp,d[0]);
    d[0].convertTo(d[0],CV_32FC1,1.0,0);

    return d[0];
}

/**
 * @brief function to compute the saliency weight map
 * @param input image is BGR image
 * @return  ouput image is scalar weight map
 */
Mat saliency(Mat s)
{

    Mat image;
    s.copyTo(image);
    Mat i,o1,o2,o3;
    cvtColor(image,image,CV_BGR2Lab);
    cv::Scalar mean=cv::mean(image);
    image.copyTo(i);
    cv::medianBlur(i,i,9);

    //image.convertTo(image,CV_32FC3,1.0/255.0,0);
    //image.copyTo(i);
    i.convertTo(i,CV_32FC3,1.0/255.0,0);
    std::vector<cv::Mat_<float> > d;
    cv::split(i,d);
    cv::add(d[0],-mean[0]/255.0,o1);
    cv::multiply(o1,o1,o1,1.0);
    cv::add(d[1],-mean[1]/255.0,o2);
    cv::multiply(o2,o2,o2,1.0);
    cv::add(d[2],-mean[2]/255.0,o3);
    cv::multiply(o3,o3,o3,1.0);
    cv::add(o1,o2,o1);
    cv::add(o1,o3,o3);
    cv::pow(o3,0.5,o1);
    //o1.convertTo(o1,CV_8U,255.0,0);
    //cv::normalize(o1,o1,0,255,cv::NORM_MINMAX);


    return o1;

}

/**
 * @brief function to calculate the local contrast weight map for image
 * @param image is input image
 * @return output image is weight map
 */
Mat LocalContrast(Mat s)
{

    Mat image;
    s.copyTo(image);
       std::vector<cv::Mat_<float> > d;

    cvtColor(image,image,CV_Lab2BGR);
      image.convertTo(image,CV_32FC3,1.0/255.0,0);
    cv::split(image,d);

     Mat blurred,diff;
     double sigma = 7, threshold = 0;
     cv::GaussianBlur(d[0], blurred, Size(), sigma, sigma);

     d[0].convertTo(d[0],CV_32F,1.0,0);

     blurred.convertTo(blurred,CV_32F,1.0,0);


     cv::absdiff(d[0],blurred,diff);
     //Mat sharpened = d[0] * (1 + amount) + blurred * (-amount);


     return diff;


}


/**
 * @brief this function call the different functions to perform image fusion
 * @param argc
 * @param argv
 * @return
 */
int main2(int argc, char *argv[])
{
  ros::init(argc, argv, "fusion");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/bottom_camera/image_raw", 1, &imageCallback);
  // image_transport::Publisher pub1 = it.advertise("/fourth_picture", 1);

    color_correction::contrast_stretching o1;
    color_correction::gray_world o2;


//    if(argc <3)
        //return -1;

  //  string dir=argv[1];
   // string name=argv[2];

   Mat a;

   //    cerr << dir+"/"+name << endl;
   //    Mat image=cv::imread(dir+"/"+name,1);

       // cv::VideoCapture cam(0);
       // if(!cam.isOpened()){
       //     std::cout<< "Camera not ready\n";
       //     return -1;
       // }
   //     clock_t begin = clock();
       while(ros::ok()){

         loop_rate.sleep();
       // cam>>image;
    //Mat image=cv::imread(dir+"/"+name,1);
    if (image.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }


    CvSize s = image.size();
    std::cout << s.width << '\n';
    std::cout << s.height << '\n';

    clock_t begin = clock();
    a.create(240, 320, CV_8UC(3));
    std::cout << "ss" << '\n';
    resize(image, a, a.size(), 0, 0, INTER_NEAREST);
    std::cout << "dd" << '\n';
    Mat src1 = o1.run(a);
    Mat src2 = o2.run2(a,6,0);

    // string name="";
    // string i1="";//argv[1]+name;
    // string i2="";//argv[1]+name;
    //imwrite(i1,src1);
    //imwrite(i2,src2);

    imshow("original image",a);
    imshow("input 1 ",src2);
    //imshow("input 2 ",src2);


    //cerr << "showing images " << endl;
        Mat c11;
    Mat c1=generateEMasks(a,a);
    c1.convertTo(c11,CV_8U,255.0,0);

    //c11=saliency(c11);




      //cv::normalize(c11,c11,255,cv::NORM_MINMAX);
      cv::Laplacian(c11,c11,c11.depth(),3,1,0);
      imshow("ddddd",c11);

    {
      // std::cout << "1" << '\n';
   Mat c2=LocalContrast(src2);
// std::cout << "2" << '\n';
    Mat c3=saliency(src1);
    // std::cout << "3" << '\n';
    Mat c4=saliency(src2);
// std::cout << "4" << '\n';
    Mat c5=generateEMasks(src1,src2);
// std::cout << "5" << '\n';
    Mat c6=generateEMasks(src2,src1);
// std::cout << "6" << '\n';
    Mat c7=localContrast(src1);
// std::cout << "7" << '\n';
    Mat c8= localContrast(src2);
// std::cout << "8" << '\n';


//uncomment for storing images
//    Mat c11,c12,c13,c14,c15,c16,c17,c18a
//    cv::normalize(c1,c11,0,1,cv::NORM_MINMAX);
//    cv::normalize(c2,c12,0,1,cv::NORM_MINMAX);
//    cv::normalize(c3,c13,0,1,cv::NORM_MINMAX);
//    cv::normalize(c4,c14,0,1,cv::NORM_MINMAX);
//    cv::normalize(c5,c15,0,1,cv::NORM_MINMAX);
//    cv::normalize(c6,c16,0,1,cv::NORM_MINMAX);
//    cv::normalize(c7,c17,0,1,cv::NORM_MINMAX);
//    cv::normalize(c8,c18,0,1,cv::NORM_MINMAX);



//    c11.convertTo(c11,CV_32F,255.0,0);
//    c12.convertTo(c12,CV_32F,255.0,0);
//    c13.convertTo(c13,CV_32F,255.0,0);
//    c14.convertTo(c14,CV_32F,255.0,0);
//    c15.convertTo(c15,CV_32F,255.0,0);
//    c16.convertTo(c16,CV_32F,255.0,0);
//    c17.convertTo(c17,CV_32F,255.0,0);
//    c18.convertTo(c18,CV_32F,255.0,0);


//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"c1.png",c11);
//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"c2.png",c12);
//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"s1.png",c13);
//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"s2.png",c14);
//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"e1.png",c15);
//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"e2.png",c16);
//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"lc1.png",c17);
//    imwrite("/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/"+name+"lc2.png",c18);


    Mat sum;
   c1.copyTo(sum);



// std::cout << "imout" << '\n';



   src1.convertTo(src1, CV_32FC3, 1.0/255.0);
   src2.convertTo(src2, CV_32FC3, 1.0/255.0);
    Mat m1,m2;
    {
   Mat result1;
   cv::add(c1,c2,sum);
   //cv::add(sum,c2,sum);

   cv::divide(c1,sum,m1);
   cv::divide(c2,sum,m2);

   LaplacianBlending lBlend(src1,src2,m1,m2,4);
   result1=lBlend.blend();

   result1.convertTo(result1,CV_8U,255.0,0);
   imshow("global contrast ",result1);
// string i1="/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/r1"+name;

    //imwrite(i1,result1);
    }


    {
           Mat result2;
   cv::add(c3,c4,sum);
   //cv::add(sum,c2,sum);

   cv::divide(c3,sum,m1);
   cv::divide(c4,sum,m2);

   LaplacianBlending lBlend(src1,src2,m1,m2,4);
   result2=lBlend.blend();

result2.convertTo(result2,CV_8U,255.0,0);
// string i1="/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/r2"+name;
 imshow("saliency",result2);
    //imwrite(i1,result2);
    }
    {
   Mat result3;
   cv::add(c5,c6,sum);
   //cv::add(sum,c2,sum);

   cv::divide(c5,sum,m1);
   cv::divide(c6,sum,m2);

   LaplacianBlending lBlend(src1,src2,m1,m2,4);
   result3=lBlend.blend();


    result3.convertTo(result3,CV_8U,255.0,0);
    // string i1="/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/r3"+name;
   imshow("exposedness",result3);
   // imwrite(i1,result3);
    }

    {
   Mat result4;
   cv::add(c7,c8,sum);
   //cv::add(sum,c2,sum);

   cv::divide(c7,sum,m1);
   cv::divide(c8,sum,m2);

   LaplacianBlending lBlend(src1,src2,m1,m2,4);
   result4=lBlend.blend();

result4.convertTo(result4,CV_8U,255.0,0);
// string i1="/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/r4"+name;
   imshow("local contrast",result4);
 //   imwrite(i1,result4);
    }

    cv::add(c1,c2,sum);
    cv::add(sum,c3,sum);
    cv::add(sum,c4,sum);
    cv::add(sum,c5,sum);
    cv::add(sum,c6,sum);
    cv::add(sum,c7,sum);
    cv::add(sum,c8,sum);

    cv::divide(c1,sum,c1);
    cv::divide(c2,sum,c2);
    cv::divide(c3,sum,c3);
    cv::divide(c4,sum,c4);
    cv::divide(c5,sum,c5);
    cv::divide(c6,sum,c6);
    cv::divide(c7,sum,c7);
    cv::divide(c8,sum,c8);





    cvtColor(c1, c1, CV_GRAY2BGR);
    cvtColor(c2, c2, CV_GRAY2BGR);
    cvtColor(c3, c3, CV_GRAY2BGR);
    cvtColor(c4, c4, CV_GRAY2BGR);
    cvtColor(c5, c5, CV_GRAY2BGR);
    cvtColor(c6, c6, CV_GRAY2BGR);
    cvtColor(c7, c7, CV_GRAY2BGR);
    cvtColor(c8, c8, CV_GRAY2BGR);

    cv::multiply(c1,src1,c1);
    cv::multiply(c3,src1,c3);
    cv::multiply(c5,src1,c5);
    cv::multiply(c7,src1,c7);
    cv::multiply(c2,src2,c2);
    cv::multiply(c4,src2,c4);
    cv::multiply(c6,src2,c6);
    cv::multiply(c8,src2,c8);


    cv::add(c1,c2,c1);
    cv::add(c1,c3,c1);
    cv::add(c1,c5,c1);
    cv::add(c1,c7,c1);
    //cv::add(c1,c2,c1);
    cv::add(c1,c4,c1);
    cv::add(c1,c6,c1);
    cv::add(c1,c8,c1);

//c1.convertTo(c1,CV_32F,255.0,0);
{


    c1.convertTo(c1,CV_8U,255.0,0);
    cv::Mat cc;
    c1.copyTo(cc);
    balance_white(cc);
    imshow("naive blend",c1);
    imshow("balance_white", cc);

    // sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", c1).toImageMsg();
    // pub1.publish(msg1);

//string i1="/media/SOFTWARES/Dropbox/Dropbox/repository/im/documents/fusion/r5"+name;
//imwrite(i1,c1);
}
    }
          cv::waitKey(1);
          k++;
          if(k==fps)
          {
          clock_t end = clock();
          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;


          k=0;
          cerr << "a " << (double)fps/elapsed_secs << endl;
          begin = clock();
          }

          ros::spinOnce();

}



}
