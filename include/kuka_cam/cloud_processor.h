#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <iostream>
#include <pcl>

class CloudProcessor 
{
	CloudProcessor();
public:
	void addCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
	void combineClouds();
	void filterWorkspace();
	void filterTable();
	void downsample();
	void estimateNormals();

	std::map<std::string, std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > >
	 
};