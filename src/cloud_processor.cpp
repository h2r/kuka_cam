#include "kuka_cam/cloud_processor.h"

namespace cloud_processor
{
  CloudProcessor::CloudProcessor(int max_deque_size,
                                 int num_clouds_to_avg) : combined_cloud_publisher() {

    max_deque_size_ = max_deque_size;
    num_clouds_to_avg_ = num_clouds_to_avg;
    if(num_clouds_to_avg_ > max_deque_size_)
    {
      max_deque_size_ = num_clouds_to_avg_;
      ROS_WARN_STREAM("Max deque size smaller than number of clouds to average, setting max_deque_size_ to " << num_clouds_to_avg_ << std::endl);
    }

    // building tf_map_ here for now.
    // this won't work for moving cameras, but let's cross that bridge later.
    ros::param::get("/pointcloud_frame_names", frame_names_);
    tf::TransformListener listener;
    for(int i = 0; i < frame_names_.size(); i++)
    {
      try
      {
        tf::StampedTransform tmp_tf;
        listener.waitForTransform("world", frame_names_[i],
                                  ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("world", frame_names_[i],
                                 ros::Time(0), tmp_tf);

        tf::transformTFToEigen(tmp_tf, tf_map_[frame_names_[i]]);
        // tf_map_[frame_names_[i]] = tmp_tf;
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }

    // output the transforms we found
    for(int i = 0; i < frame_names_.size(); i++)
    {
      // ROS_INFO_STREAM(frame_names_[i] << " position: " << tf_map_[frame_names_[i]].getOrigin()[0] << ", " << tf_map_[frame_names_[i]].getOrigin()[1] << ", " << tf_map_[frame_names_[i]].getOrigin()[2]);
      // ROS_INFO_STREAM(frame_names_[i] << " rotation: " << tf_map_[frame_names_[i]].getRotation()[0] << ", " << tf_map_[frame_names_[i]].getRotation()[1] << ", " << tf_map_[frame_names_[i]].getRotation()[2] <<  ", " << tf_map_[frame_names_[i]].getRotation()[3] );
      ROS_INFO_STREAM(std::setprecision(20) << frame_names_[i] << " translation: "
                                     << tf_map_[frame_names_[i]].translation()[0] << ", "
                                     << tf_map_[frame_names_[i]].translation()[1] << ", "
                                     << tf_map_[frame_names_[i]].translation()[2]);

      ROS_INFO_STREAM(std::setprecision(20) << frame_names_[i] << " rotation: "
                                     << tf_map_[frame_names_[i]].rotation()(0) << ", "
                                     << tf_map_[frame_names_[i]].rotation()(1) << ", "
                                     << tf_map_[frame_names_[i]].rotation()(2) << ", "
                                     << tf_map_[frame_names_[i]].rotation()(3));
    }

  }
  void CloudProcessor::addCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    std::string camera_name = msg->header.frame_id;
    double timestamp = msg->header.stamp.toSec();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // push cloud, timestamp back.
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> my_pair;
    my_pair.first = cloud;
    my_pair.second = timestamp;
    cloud_map_[camera_name].push_back(my_pair);

    // pop front if deque is full
    if (cloud_map_[camera_name].size() > max_deque_size_)
    {
      cloud_map_[camera_name].pop_front();
    }

    // ROS_INFO_STREAM("----------");
    // ROS_INFO_STREAM(std::setprecision(20) << timestamp << std::endl);
    // ROS_INFO_STREAM(camera_name << std::endl);
    // ROS_INFO_STREAM(cloud_map_[camera_name].size());

  }
  void CloudProcessor::combineClouds(){

    // // combined_cloud_->resize(0);
    combined_cloud_->clear();
    combined_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < tf_map_.size(); i++)
    {
      // TODO: make sure this cloud isn't stale.
      if (cloud_map_[frame_names_[i]].size() > 0)
      {
        // TODO: averaging.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud( *cloud_map_[frame_names_[i]].back().first,
                                  *cloud_out,
                                  tf_map_[frame_names_[i]]);
        *combined_cloud_+=*cloud_out;
      }
    }


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
  void CloudProcessor::publishCombined(const ros::TimerEvent& event){

    // TODO: is this the right place for this?
    combineClouds();

    combined_cloud_->header.frame_id="world";
    combined_cloud_->header.stamp= last_combined_pub_time_.toSec();
    combined_cloud_publisher.publish(combined_cloud_);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudProcessor::getCombinedClouds(){
    return combined_cloud_;
  }
}
