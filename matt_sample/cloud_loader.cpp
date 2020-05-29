#include "occgrid_common/cloud_loader.h"
namespace cloud_loader
{
  int getSecondView(int view)
  {
    return (view + 54) % 360;
  }
  // TODO: Don't need to pass ptrs by reference, right?
  template <typename CloudPointType>
  void assignPointsToNormals(pcl::PointCloud<pcl::PointNormal>::Ptr& norm,
    typename pcl::PointCloud<CloudPointType>::Ptr cloud)
  {
    if (!norm->points.size() == cloud->points.size())
    {
      std::cerr << "Number of normals (" << norm->points.size() << ") is not equal to number of points (" <<
        cloud->points.size() << ")." << std::endl;
      throw;
    }
    for(size_t pt_ind = 0; pt_ind<cloud->points.size(); pt_ind++)
    {
      norm->points[pt_ind].x = cloud->points[pt_ind].x;
      norm->points[pt_ind].y = cloud->points[pt_ind].y;
      norm->points[pt_ind].z = cloud->points[pt_ind].z;
    }
  }
  /*template <typename VoxPointType>
  void voxelizeCloud(typename pcl::PointCloud<VoxPointType>::Ptr& cloud, float voxel_size)
  {
    pcl::VoxelGrid<VoxPointType> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud);
    return;
  }*/

  template <typename PointType>
  CloudLoader<PointType>::CloudLoader(boost::shared_ptr<bigbird_calib::Calibration>& obj_calib_data,
    const std::string& obj_dir, const std::string& object_name, const std::string& occluded_cloud_dir, bool downsample)
  {
    init(obj_calib_data, obj_dir, object_name, occluded_cloud_dir, downsample);
  }
  template <typename PointType>
  CloudLoader<PointType>::CloudLoader(const std::string& calib_dir, const std::string& obj_dir,
    std::string& object_name, const std::string& occluded_cloud_dir, bool downsample)
  {
    std::map<std::string, boost::shared_ptr<bigbird_calib::Calibration> > calib_data =
      bigbird_calib::readCalibration(calib_dir);
    boost::shared_ptr<bigbird_calib::Calibration> obj_calib_data = calib_data[object_name];
    init(obj_calib_data, obj_dir, object_name, occluded_cloud_dir, downsample);
  }

  template <typename PointType>
  void CloudLoader<PointType>::init(boost::shared_ptr<bigbird_calib::Calibration>& obj_calib_data,
    const std::string& obj_dir, const std::string& object_name, const std::string& occluded_cloud_dir, bool downsample)
  {
    obj_calib_data_ = obj_calib_data;
    obj_dir_ = obj_dir;
    object_name_ = object_name;
    occluded_cloud_dir_ = occluded_cloud_dir;
    downsample_ = downsample;

    ne_radius_ = 0.03;
    voxel_size_ = 0.002;
    typename pcl::search::KdTree<PointType>::Ptr ne_tree (new pcl::search::KdTree<PointType>());
    ne_tree_ = ne_tree;
    ne_.setSearchMethod(ne_tree_);
  }

  template <typename PointType>
  typename pcl::PointCloud<PointType>::Ptr CloudLoader<PointType>::getCombinedCloud(int cam_num, int cloud_num)
  {
    typename pcl::PointCloud<PointType>::Ptr cloud_1 = loadAndCalibrateCloud(cam_num, cloud_num);
    int second_cloud_view = getSecondView(cloud_num);
    typename pcl::PointCloud<PointType>::Ptr cloud_2 = loadAndCalibrateCloud(cam_num, second_cloud_view);
    /*if (downsample_)
    {
      voxelizeCloud<PointType>(cloud_1, voxel_size_);
      voxelizeCloud<PointType>(cloud_2, voxel_size_);
    }*/
    *cloud_1 += *cloud_2;
    return cloud_1;
  }
  template <typename PointType>
  void CloudLoader<PointType>::getCloudAndNormals(typename pcl::PointCloud<PointType>::Ptr& cloud_assign,
    pcl::PointCloud<pcl::PointNormal>::Ptr& point_norm_assign, int cam_num, int cloud_num)
  {
    typename pcl::PointCloud<PointType>::Ptr cloud = loadAndCalibrateCloud(cam_num, cloud_num);
    pcl::PointCloud<pcl::PointNormal>::Ptr norm = estimateNormals(cloud, cam_num, cloud_num);

    cloud_assign = cloud;
    point_norm_assign = norm;
    return;
  }
  template <typename PointType>
  void CloudLoader<PointType>::getCombinedCloudAndNormals(typename pcl::PointCloud<PointType>::Ptr& cloud_assign,
    pcl::PointCloud<pcl::PointNormal>::Ptr& point_norm_assign, int cam_num, int cloud_num)
  {
    typename pcl::PointCloud<PointType>::Ptr cloud_1 = loadAndCalibrateCloud(cam_num, cloud_num);
    pcl::PointCloud<pcl::PointNormal>::Ptr norm_1 = estimateNormals(cloud_1, cam_num, cloud_num);

    int second_cloud_view = getSecondView(cloud_num);
    typename pcl::PointCloud<PointType>::Ptr cloud_2 = loadAndCalibrateCloud(cam_num, second_cloud_view);
    pcl::PointCloud<pcl::PointNormal>::Ptr norm_2 = estimateNormals(cloud_2, cam_num, second_cloud_view);

    /*if (downsample_)
    {
      voxelizeCloudAndNormal(cloud_1, norm_1);
      voxelizeCloudAndNormal(cloud_2, norm_2);
    }*/
    *cloud_1 += *cloud_2;
    *norm_1 += *norm_2;
    cloud_assign = cloud_1;
    point_norm_assign = norm_1;
    return;
  }
  template <typename PointType>
  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudLoader<PointType>::getOccludedCloud(int cam_num, int cloud_num)
  {
    if (occluded_cloud_dir_ == "")
    {
      std::cerr << "Requested occluded cloud but did not provide directory to cloud loader." << std::endl;
      throw;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr occluded_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string occluded_pcd_path = occluded_cloud_dir_ + std::to_string(cam_num) + "_" + std::to_string(cloud_num) +
      ".pcd";
    pcl::io::loadPCDFile (occluded_pcd_path, *occluded_cloud);
    return occluded_cloud;
  }

  template <typename PointType>
  typename pcl::PointCloud<PointType>::Ptr CloudLoader<PointType>::loadAndCalibrateCloud(int cam_num, int cloud_num)
  {
    typename pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    std::string cloud_name = obj_dir_ + '/' + object_name_ + "/clouds/NP" + std::to_string(cam_num) + "_" +
      std::to_string(cloud_num) + ".pcd";
    pcl::io::loadPCDFile (cloud_name, *cloud);
    Eigen::Matrix4f cam_to_table = bigbird_calib::camToTable(obj_calib_data_, cam_num, cloud_num);
    pcl::transformPointCloud(*cloud, *cloud, cam_to_table);
    return cloud;
  }

  /*template <typename PointType>
  void CloudLoader<PointType>::voxelizeCloudAndNormal(typename pcl::PointCloud<PointType>::Ptr& cloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr& normals)
  {
    voxelizeCloud<PointType>(cloud, voxel_size_);
    voxelizeCloud<pcl::PointNormal>(normals, voxel_size_);
  }*/

  template <typename PointType>
  pcl::PointCloud<pcl::PointNormal>::Ptr CloudLoader<PointType>::estimateNormals(
    typename pcl::PointCloud<PointType>::Ptr cloud, int cam_num, int cloud_num)
  {
    Eigen::Matrix4f cam_to_table = bigbird_calib::camToTable(obj_calib_data_, cam_num, cloud_num);
    
    ne_.setViewPoint(cam_to_table(0, 3), cam_to_table(1, 3), cam_to_table(2, 3));
    ne_.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr norm (new pcl::PointCloud<pcl::PointNormal>);
    ne_.setRadiusSearch(ne_radius_);
    ne_.compute(*norm);

    assignPointsToNormals<PointType>(norm, cloud);
    return norm;
  }

  template class CloudLoader<pcl::PointXYZRGB>;
  template class CloudLoader<pcl::PointXYZRGBA>;
  template class CloudLoader<pcl::PointXYZ>;
} //namespace cloud_loader