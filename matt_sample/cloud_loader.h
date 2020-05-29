#ifndef _OCCGRID_COMMON_CLOUD_LOADER_H_
#define _OCCGRID_COMMON_CLOUD_LOADER_H_

#include <iostream>
#include <map>
#include <string>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "occgrid_common/bigbird_calib.h"

namespace cloud_loader
{
  int getSecondView(int view);
  template <typename CloudPointType> void assignPointsToNormals(pcl::PointCloud<pcl::PointNormal>::Ptr& norm,
    typename pcl::PointCloud<CloudPointType>::Ptr cloud);
  /*template <typename VoxPointType> void voxelizeCloud(typename pcl::PointCloud<VoxPointType>::Ptr& cloud,
    float voxel_size);*/

  template <typename PointType>
  class CloudLoader
  {
  public:
    CloudLoader(boost::shared_ptr<bigbird_calib::Calibration>& obj_calib_data, const std::string& obj_dir,
      const std::string& object_name, const std::string& occluded_cloud_dir="", bool downsample=false);
    CloudLoader(const std::string& calib_dir, const std::string& obj_dir, std::string& object_name,
      const std::string& occluded_cloud_dir="", bool downsample=false);

    typename pcl::PointCloud<PointType>::Ptr getCombinedCloud(int cam_num, int cloud_num);
    void getCombinedCloudAndNormals(typename pcl::PointCloud<PointType>::Ptr& cloud_assign,
      pcl::PointCloud<pcl::PointNormal>::Ptr& point_norm_assign, int cam_num, int cloud_num);
    void getCloudAndNormals(typename pcl::PointCloud<PointType>::Ptr& cloud_assign,
      pcl::PointCloud<pcl::PointNormal>::Ptr& point_norm_assign, int cam_num, int cloud_num);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getOccludedCloud(int cam_num, int cloud_num);
  private:
    void init(boost::shared_ptr<bigbird_calib::Calibration>& obj_calib_data, const std::string& obj_dir,
      const std::string& object_name, const std::string& occluded_cloud_dir, bool downsample);
    typename pcl::PointCloud<PointType>::Ptr loadAndCalibrateCloud(int cam_num, int cloud_num);
    /*void voxelizeCloudAndNormal(typename pcl::PointCloud<PointType>::Ptr& cloud,
      pcl::PointCloud<pcl::PointNormal>::Ptr& normals);*/
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(typename pcl::PointCloud<PointType>::Ptr cloud, int cam_num,
      int cloud_num);

    float ne_radius_;
    float voxel_size_;
    bool downsample_;

    boost::shared_ptr<bigbird_calib::Calibration> obj_calib_data_;
    std::string obj_dir_;
    std::string object_name_;
    std::string occluded_cloud_dir_;

    typename pcl::NormalEstimation<PointType, pcl::PointNormal> ne_;
    typename pcl::search::KdTree<PointType>::Ptr ne_tree_;
  };
} //namespace cloud_loader

#endif //_OCCGRID_COMMON_CLOUD_LOADER_H_