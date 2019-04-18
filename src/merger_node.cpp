#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

void callback1(const sensor_msgs::PointCloud2ConstPtr& cloud1)
{
  std::cout << "got 1 cloud" << std::endl;
}

void callback2(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2)
{
  std::cout << "got 2 clouds" << std::endl;
}

void callback3(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2, const sensor_msgs::PointCloud2ConstPtr& cloud3)
{
  std::cout << "got 3 clouds" << std::endl;
}

void callback4(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2, const sensor_msgs::PointCloud2ConstPtr& cloud3,
               const sensor_msgs::PointCloud2ConstPtr& cloud4)
{
  std::cout << "got 4 clouds" << std::endl;
}

void callback5(const sensor_msgs::PointCloud2ConstPtr& cloud1, const sensor_msgs::PointCloud2ConstPtr& cloud2, const sensor_msgs::PointCloud2ConstPtr& cloud3,
               const sensor_msgs::PointCloud2ConstPtr& cloud4, const sensor_msgs::PointCloud2ConstPtr& cloud5)
{
  std::cout << "got 5 clouds" << std::endl;
}

int main(int argc, char **argv)
{

  ROS_INFO("HELLO");

  ros::init(argc, argv, "cloud listener");
  ros::NodeHandle nh;

  bool cam_1_on, cam_2_on, cam_3_on, cam_4_on, cam_5_on;
  ros::param::get("/cam_1_on", cam_1_on);
  ros::param::get("/cam_2_on", cam_2_on);
  ros::param::get("/cam_3_on", cam_3_on);
  ros::param::get("/cam_4_on", cam_4_on);
  ros::param::get("/cam_5_on", cam_5_on);

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1(nh, "/cam_1/depth/color/points", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2(nh, "/cam_2/depth/color/points", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_3(nh, "/cam_3/depth/color/points", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_4(nh, "/cam_4/depth/color/points", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_5(nh, "/cam_5/depth/color/points", 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
                                                          sensor_msgs::PointCloud2, 
                                                          sensor_msgs::PointCloud2, 
                                                          sensor_msgs::PointCloud2, 
                                                          sensor_msgs::PointCloud2> MySyncPolicyType;

  MySyncPolicyType my_sync_policy(1000);
  ros::Duration my_duration(10.0);
  my_sync_policy.setMaxIntervalDuration(my_duration);

  message_filters::Synchronizer<MySyncPolicyType> sync( static_cast<const MySyncPolicyType &>(my_sync_policy), sub_1, sub_2, sub_3, sub_4, sub_5);
  sync.registerCallback(boost::bind(&callback5, _1, _2, _3, _4, _5));

  ROS_INFO("GOODBYE");

  ros::spin();
  return 0;
}