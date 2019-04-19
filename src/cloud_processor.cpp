#include "kuka_cam/cloud_processor.h"

namespace cloud_processor
{
  CloudProcessor::CloudProcessor(int max_deque_size, int num_clouds_to_avg){
    max_deque_size_ = max_deque_size;
    num_clouds_to_avg_ = num_clouds_to_avg;
  }
  void CloudProcessor::addCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    ROS_INFO_STREAM(msg->header.stamp);
    ROS_INFO_STREAM(msg->header.frame_id << std::endl); 
  }
  void CloudProcessor::combineClouds(){

  }
  void CloudProcessor::filterWorkspace(){

  }
  void CloudProcessor::filterTable(){

  }
  void CloudProcessor::downsample(){

  }
  void CloudProcessor::estimateNormals(){

  }
  void CloudProcessor::filterOutliers(){

  }
  void CloudProcessor::publishCombined(){

  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudProcessor::getCombinedClouds(){
    return combined_cloud_;
  } 
}