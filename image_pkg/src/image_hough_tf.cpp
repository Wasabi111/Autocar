#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vector>
#include <iostream>

static const std::string COLOR_OPENCV_WINDOW = "Color Image";
static const std::string DEPTH_OPENCV_WINDOW = "Depth Image";
static const std::string EDGE_OPENCV_WINDOW = "Hough Transform";

using namespace std;

void Color_Image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_color_ptr;
  try
  {
    cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // The original color image
  cv::Mat color_pic = cv_color_ptr->image;
  cv::namedWindow(COLOR_OPENCV_WINDOW);
  cv::imshow(COLOR_OPENCV_WINDOW, color_pic);


  /////////////////////////////////////////////////////////////////////////////
  // Image processing /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  cv::Mat dst;
  cv::Mat cdst;
  cv::Mat gray_pic;
  cv::cvtColor(color_pic, gray_pic, CV_BGR2GRAY);

  // Edge detection (Canny Edge Detection)
  int lowThreshold = 100;
  int ratio = 3;
  int kernel_size = 3;
  cv::Canny(gray_pic, dst, lowThreshold, lowThreshold*ratio, kernel_size);

  // Standard Hough Transform
  vector<cv::Vec2f> lines; // will hold the results of the detection
  double rho = 1;
  double theta = CV_PI/180;
  int hough_threshold = 80;

  cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
  cv::HoughLines(dst, lines, rho, theta, hough_threshold, 0, 0 ); // runs the actual detection
  cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);

  // Draw the lines
  for( size_t i = 0; i < lines.size(); i++ )
  {
      float rho = lines[i][0], theta = lines[i][1];

      // Set the rho bound for lane detection
      float rholowbound = 140;
      float rhohighbound = 160;
      if (rho > rholowbound && rho < rhohighbound)
      {
        // Set the theta bound for left lane detection (red line)
        float thetaleftlowbound = 80*CV_PI/180;
        float thetalefthighbound = 90*CV_PI/180;
        if (theta > thetaleftlowbound && theta < thetalefthighbound)
        {
          cv::Point pt1, pt2;
          double a1 = cos(theta), b1 = sin(theta);
          double x1 = a1*rho, y1 = b1*rho;
          pt1.x = cvRound(x1 + 1000*(-b1));
          pt1.y = cvRound(y1 + 1000*(a1));
          pt2.x = cvRound(x1 - 1000*(-b1));
          pt2.y = cvRound(y1 - 1000*(a1));
          cv::line(cdst, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
        }

        // Set the theta bound for right lane detection (green line)
        float thetarightlowbound = 90*CV_PI/180;
        float thetarightthighbound = 100*CV_PI/180;
        if (theta > thetarightlowbound && theta < thetarightthighbound)
        {
          cv::Point pt3, pt4;
          double a2 = cos(theta), b2 = sin(theta);
          double x2 = a2*rho, y2 = b2*rho;
          pt3.x = cvRound(x2 + 1000*(-b2));
          pt3.y = cvRound(y2 + 1000*(a2));
          pt4.x = cvRound(x2 - 1000*(-b2));
          pt4.y = cvRound(y2 - 1000*(a2));
          cv::line(cdst, pt3, pt4, cv::Scalar(0,255,0), 3, cv::LINE_AA);

          
        }
      }
  }

  /*// Probabilistic Line Transform
  int hough_threshold2 = 30;
  int minLinLength = 50;
  int maxLineGap;
  vector<cv::Vec4i> linesP; // will hold the results of the detection
  cv::HoughLinesP(dst, linesP, rho, theta, hough_threshold2, minLinLength, maxLineGap); // runs the actual detection
  // Draw the lines
  for( size_t i = 0; i < linesP.size(); i++ )
  {
      cv::Vec4i l = linesP[i];
      cv::line( cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
  }*/


  // Show results
  cv::imshow(EDGE_OPENCV_WINDOW, cdst);

  cv::waitKey(3);
  // Output modified video stream
  /*ros::NodeHandle nh;
  image_transport::ImageTransport ith(nh);
  image_transport::Publisher colo_image_pub;
  colo_image_pub = ith.advertise("/image_converter/output_video", 1);
  colo_image_pub.publish(cv_ptr->toImageMsg());*/
}

void Depth_Image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv::namedWindow(DEPTH_OPENCV_WINDOW);
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Print out the depth information
    int depth = cv_depth_ptr->image.at<short int>(cv::Point(240,320));//you can change 240,320 to your interested pixel
    ROS_INFO("Depth: %d", depth);

    // Update GUI Window
    cv::imshow(DEPTH_OPENCV_WINDOW, cv_depth_ptr->image);
    cv::waitKey(3);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_process");
  // ImageConverter ic;
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  // Create a color image subscriber
  image_transport::Subscriber color_image_sub;

  // Subscribe to color input video feed and publish output video feed
  color_image_sub = it.subscribe("/camera/color/image_raw", 1, Color_Image_Callback);



  // Create a depth image subscriber
  image_transport::Subscriber depth_image_sub;

  // Subscribe to depth input video feed
  //depth_image_sub = it.subscribe("/camera/depth/image_rect_raw", 1, Depth_Image_Callback);



  ros::spin();
  return 0;
}
