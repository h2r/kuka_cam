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
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

typedef pcl::PointXYZ InputPointType;
typedef pcl::PointNormal OutputPointType;

namespace cloud_processor
{
  // template<class OutputPointType>
  class CloudProcessor
  {
  public:
    CloudProcessor(int max_deque_size, int num_clouds_to_avg);
    void addCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

    // void combineClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
    void combineClouds(pcl::PointCloud<pcl::PointNormal>::Ptr &output);

    void filterWorkspace();
    void filterTable(pcl::PointCloud<OutputPointType>::Ptr &output);
    void downsample();
    void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                         pcl::PointCloud<pcl::PointNormal>::Ptr &output);
    void filterOutliers();
    void publishCombined(const ros::TimerEvent& event);
    pcl::PointCloud<OutputPointType>::Ptr getCombinedClouds();
    ros::Publisher combined_cloud_publisher;

  private:
    std::map<std::string, std::deque<std::pair<pcl::PointCloud<InputPointType>::Ptr, double> > > cloud_map_;
    std::map<std::string, Eigen::Affine3d> tf_map_;
    int max_deque_size_;
    int num_clouds_to_avg_;
    pcl::PointCloud<OutputPointType>::Ptr combined_cloud_ = boost::shared_ptr<pcl::PointCloud<OutputPointType> >(new pcl::PointCloud<OutputPointType>);
    std::vector<std::string> frame_names_;
  };
}

#endif
