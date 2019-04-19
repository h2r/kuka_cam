#ifndef KUKA_CAM_CLOUD_PROCESSOR
#define KUKA_CAM_CLOUD_PROCESSOR

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

namespace cloud_processor
{
  class CloudProcessor 
  {
    CloudProcessor(int max_deque_size, int num_clouds_to_avg);
  public:
    void addCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    void combineClouds();
    void filterWorkspace();
    void filterTable();
    void downsample();
    void estimateNormals();
    void filterOutliers();
    void publishCombined();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCombinedClouds(); 

  private:
    std::map<std::string, std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > > cloud_map_;
    std::map<std::string, Eigen::Matrix4d> tf_map_;
    int max_deque_size_;
    int num_clouds_to_avg_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud_;
  };
}

#endif