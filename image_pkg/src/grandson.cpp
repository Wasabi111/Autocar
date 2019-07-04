#include <ros/ros.h>
#include "std_msgs/Int64.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string COLOR_OPENCV_WINDOW = "Color Image";
static const std::string DEPTH_OPENCV_WINDOW = "Depth Image";
ros::Subscriber status_sub;
int offset = 40;
int window_size_x = 9;
int window_size_y = 9;
int status = 0;
int prev_status = -1;
unsigned short left_depth = 0;
unsigned short center_depth = 0;
unsigned short right_depth = 0;

enum Status
{
    Stop = 0,
    Straight = 1,
    TurnRight = 2,
    TurnLeft = 3
};

void color_image_callback(const sensor_msgs::ImageConstPtr& msg)
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
    int rows = cv_color_ptr->image.rows;
    int cols = cv_color_ptr->image.cols;
    
    cv::Point pt1(offset, rows/2 - 30);
    cv::Point pt2(offset + window_size_x, rows/2 - 30 + window_size_y);
    cv::rectangle(cv_color_ptr->image, pt1, pt2, cv::Scalar(255, 0, 0), 3);

    cv::Point pt3((cols-window_size_x)/2, rows/2 - 30);
    cv::Point pt4((cols-window_size_x)/2 + window_size_x, rows/2 - 30 + window_size_y);
    cv::rectangle(cv_color_ptr->image, pt3, pt4, cv::Scalar(255, 0, 0), 3);

    cv::Point pt5(cols-window_size_x-offset, rows/2 - 30);
    cv::Point pt6(cols-window_size_x-offset + window_size_x, rows/2 - 30 + window_size_y);
    cv::rectangle(cv_color_ptr->image, pt5, pt6, cv::Scalar(255, 0, 0), 3);
    // Update GUI Window
    cv::imshow(COLOR_OPENCV_WINDOW, cv_color_ptr->image);
    cv::waitKey(3);
}

unsigned short depth_calculation(cv_bridge::CvImagePtr cv_depth_ptr, int left_upper_x, int left_upper_y, int window_size_x, int window_size_y)
{
    //std::vector<unsigned short> depth_vector;
    unsigned int sum = 0;
    int count = 0;
    for(int i = left_upper_y;i < left_upper_y + window_size_y; i++)
    {
        for(int j = left_upper_x;j < left_upper_x + window_size_x; j++)
        {
            unsigned short depth = cv_depth_ptr->image.at<unsigned short>(i, j);
            if(depth != 0)
            {
                    count++;
                    //depth_vector.push_back(depth);
                sum += depth;
            }
        }
    }

    if(count != 0){
        //std::sort(depth_vector.begin(), depth_vector.end());
        return (unsigned short)(sum/count);
    }
    else{
        return 0;
    }
}

void depth_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_depth_ptr;
    
    //cv::namedWindow(DEPTH_OPENCV_WINDOW);
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    int rows = cv_depth_ptr->image.rows;
    int cols = cv_depth_ptr->image.cols;

    // Print out the depth information
    left_depth = depth_calculation(cv_depth_ptr, offset, rows/2 - 30, window_size_x, window_size_y);
    center_depth = depth_calculation(cv_depth_ptr, (cols-window_size_x)/2, rows/2 - 30, window_size_x, window_size_y);
    right_depth = depth_calculation(cv_depth_ptr, cols-window_size_x-offset, rows/2 - 30, window_size_x, window_size_y);
    // ROS_INFO("Left: %u, Center: %u, Right: %u", left_depth, center_depth, right_depth);

    // Update GUI Window
    //cv::imshow(DEPTH_OPENCV_WINDOW, cv_depth_ptr->image);
    //cv::waitKey(3);
}

void status_callback(const std_msgs::Int64 &msg)
{
    if(msg.data == 0){
        status = Stop;
    }
    else
    {
        status = msg.data;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    // ImageConverter ic;
    ros::NodeHandle n;
    ros::Publisher motor_pub = n.advertise<std_msgs::Int64>("/motor", 100);
    ros::Publisher steer_pub = n.advertise<std_msgs::Int64>("/steer", 100);
    ros::Publisher status_pub = n.advertise<std_msgs::Int64>("/status", 100);
    ros::Rate loop_rate(30);
    image_transport::ImageTransport it(n);

    // Create a color image subscriber
    //image_transport::Subscriber color_image_sub;

    // Subscribe to color input video feed and publish output video feed
    //color_image_sub = it.subscribe("/camera/color/image_raw", 1, color_image_callback);

    
    // Create a depth image subscriber
    image_transport::Subscriber depth_image_sub;

    // Subscribe to depth input video feed
    depth_image_sub = it.subscribe("/camera/depth/image_rect_raw", 1, depth_image_callback);
    
    // Check the status
    status_sub = n.subscribe("status",1000, status_callback);

    while(ros::ok())
    {
        std_msgs::Int64 msg;
        if(status == TurnRight || status == TurnLeft)
        {
            msg.data = 6325;
            motor_pub.publish(msg);
            if(center_depth > 7500) // go back to Straight
            {
                msg.data = Straight;
                status_pub.publish(msg);
                status = Straight;
                msg.data = 6000;
                steer_pub.publish(msg);
            }
        }
        else if(status == Straight)
        {
            msg.data = 6350;
            motor_pub.publish(msg);
            prev_status = status;
            if(center_depth < 3500 && center_depth != 0)
            {
                // Turn 
                // if(right_depth > 1000 + left_depth)
                // {   
                msg.data = TurnRight;
                status_pub.publish(msg);
                status = TurnRight;
                // Turn right
                msg.data = 6800;
                steer_pub.publish(msg);
                // 
                // else if(left_depth > 1000 + right_depth)
                // {
                //     msg.data = TurnLeft;
                //     status_pub.publish(msg);
                //     status = TurnLeft;
                //     // Turn left
                //     msg.data = 5200;
                //     steer_pub.publish(msg);
                // }
            }
            else if(right_depth > 800 + left_depth)
            {
                // Turn right a little bit
                msg.data = 6200;
                steer_pub.publish(msg);
            }
            else if(left_depth > 800 + right_depth)
            {
                // Turn left a little bit
                msg.data = 5800;
                steer_pub.publish(msg);
            }
            else
            {
                msg.data = 6000;
                steer_pub.publish(msg);
            }
        }
        else
        {
            // Stop
            msg.data = 6000;
            motor_pub.publish(msg);
            prev_status = status;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
