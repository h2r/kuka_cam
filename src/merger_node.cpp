#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <iostream>


void callback_placeholder(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->header.stamp);
  ROS_INFO_STREAM(msg->header.frame_id << std::endl); 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cloud listener");
  ros::NodeHandle nh;

  bool cam_1_on, cam_2_on, cam_3_on, cam_4_on, cam_5_on;
  ros::param::get("/cam_1_on", cam_1_on);
  ros::param::get("/cam_2_on", cam_2_on);
  ros::param::get("/cam_3_on", cam_3_on);
  ros::param::get("/cam_4_on", cam_4_on);
  ros::param::get("/cam_5_on", cam_5_on);

  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  ros::Subscriber sub_3;
  ros::Subscriber sub_4;
  ros::Subscriber sub_5;

  if (cam_1_on)
  {
    sub_1 = nh.subscribe("/cam_1/depth/color/points", 10, callback_placeholder);  
  }

  if (cam_2_on)
  {
    sub_2 = nh.subscribe("/cam_2/depth/color/points", 10, callback_placeholder);
  }

  if (cam_3_on)
  {
    sub_3 = nh.subscribe("/cam_3/depth/color/points", 10, callback_placeholder);
  }

  if (cam_4_on)
  {
    sub_4 = nh.subscribe("/cam_4/depth/color/points", 10, callback_placeholder);

  }
  if (cam_5_on)
  {
    sub_5 = nh.subscribe("/cam_5/depth/color/points", 10, callback_placeholder);
  }  

  ros::spin();
  return 0;
}