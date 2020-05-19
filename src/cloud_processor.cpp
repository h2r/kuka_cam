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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> my_pair;
    my_pair.first = cloud;
    my_pair.second = timestamp;
    cloud_map_[camera_name].push_back(my_pair);

    if (cloud_map_[camera_name].size() > max_deque_size_)
    {
      cloud_map_[camera_name].pop_front();
    }

    ROS_INFO_STREAM("----------");
    ROS_INFO_STREAM(std::setprecision(20) << timestamp << std::endl);
    ROS_INFO_STREAM(camera_name << std::endl);
    ROS_INFO_STREAM(cloud_map_[camera_name].size());

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudProcessor::getCombinedClouds(){
    return combined_cloud_;
  }
}
