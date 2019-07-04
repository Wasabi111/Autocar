#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

void Accel_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Linear Acceleration x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
}

void Ang_Vel_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Angular Velocity x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle n;

  // Create two subscribers to listen linear acceleration and angular velocity data
  ros::Subscriber accel_sub = n.subscribe("/camera/accel/sample", 1000, Accel_Callback);
  ros::Subscriber ang_vel_sub = n.subscribe("/camera/gyro/sample", 1000, Ang_Vel_Callback);

  ros::spin();

  return 0;
}
