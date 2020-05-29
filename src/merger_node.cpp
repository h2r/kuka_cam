#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "kuka_cam/cloud_processor.h"

// void callback_placeholder(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//   ROS_INFO_STREAM(msg->header.stamp);
//   ROS_INFO_STREAM(msg->header.frame_id << std::endl);
// }

typedef pcl::PointNormal myPointType;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud listener");
  ros::NodeHandle nh;

  std::vector<std::string> topic_names;
  ros::param::get("/pointcloud_topic_names", topic_names);
  ROS_INFO_STREAM("Subscribing to " << topic_names.size() << " topics.");

  int max_deque_size = 5;
  int num_clouds_to_avg = 1;
  double combined_pub_freq = 10.0;

  cloud_processor::CloudProcessor<myPointType> cp(max_deque_size, num_clouds_to_avg);

  std::vector<ros::Subscriber> subscribers;
  for(int i = 0; i < topic_names.size(); i++)
  {
    ROS_INFO_STREAM("Subscribing to " << topic_names[i]);
    subscribers.push_back(nh.subscribe(topic_names[i], 10, &cloud_processor::CloudProcessor<myPointType>::addCloud, &cp));
  }

  cp.combined_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cloud_combined", 1);
  ros::Timer timer_pub = nh.createTimer(ros::Duration( 1.0 / combined_pub_freq) , &cloud_processor::CloudProcessor<myPointType>::publishCombined, &cp);

  ros::spin();
  return 0;
}
