#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string COLOR_OPENCV_WINDOW = "Color Image";
static const std::string DEPTH_OPENCV_WINDOW = "Depth Image";

void Color_Image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_color_ptr;
  cv::namedWindow(COLOR_OPENCV_WINDOW);
  try
  {
    cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  // Image processing

  // Update GUI Window
  cv::imshow(COLOR_OPENCV_WINDOW, cv_color_ptr->image);
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
  ros::init(argc, argv, "image_converter");
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
  depth_image_sub = it.subscribe("/camera/depth/image_rect_raw", 1, Depth_Image_Callback);



  ros::spin();
  return 0;
}
