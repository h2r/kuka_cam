#include "kuka_cam/cloud_processor.h"

namespace cloud_processor
{
  CloudProcessor::CloudProcessor(int max_deque_size, int num_clouds_to_avg){
    max_deque_size_ = max_deque_size;
    num_clouds_to_avg_ = num_clouds_to_avg;
    if(num_clouds_to_avg_ > max_deque_size_)
    {
      max_deque_size_ = num_clouds_to_avg_;
      ROS_WARN_STREAM("Max deque size smaller than number of clouds to average, setting max_deque_size_ to " << num_clouds_to_avg_ << std::endl);
    }
  }
  void CloudProcessor::addCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    std::string camera_name = msg->header.frame_id;
    double timestamp = msg->header.stamp.toSec();

    // TODO: fix warning "failed to field matching field rgb."
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
    pcl::fromROSMsg(*msg, *cloud);

    ROS_INFO_STREAM(std::setprecision(20) << timestamp << std::endl);
    ROS_INFO_STREAM(camera_name << std::endl);

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
