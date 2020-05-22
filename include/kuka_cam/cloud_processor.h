#ifndef KUKA_CAM_CLOUD_PROCESSOR
#define KUKA_CAM_CLOUD_PROCESSOR

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
 #include <tf_conversions/tf_eigen.h>

#include <sstream>
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace cloud_processor
{
  class CloudProcessor
  {
  public:
    CloudProcessor(int max_deque_size, int num_clouds_to_avg);
    void addCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    void combineClouds();
    void filterWorkspace();
    void filterTable();
    void downsample();
    void estimateNormals();
    void filterOutliers();
    void publishCombined(const ros::TimerEvent& event);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCombinedClouds();
    ros::Publisher combined_cloud_publisher;

  private:
    std::map<std::string, std::deque<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> > > cloud_map_;
    std::map<std::string, Eigen::Affine3d> tf_map_;
    int max_deque_size_;
    int num_clouds_to_avg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::string> frame_names_;
  };
}

#endif
