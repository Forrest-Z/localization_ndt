#ifndef POINT_CLOUD_MAP_H_
#define POINT_CLOUD_MAP_H_

using namespace std;

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
//#include "NNGridTable.h"
#include "Timer.h"

class PointCloudMap {
private:
  static const int MAX_POINT_NUM=10000000;             // globalMapの最大点数

  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud;         // ファイル入力用

  double LeafSize; // フィルタサイズ

public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;  // フィルター済み地図

  PointCloudMap() : LeafSize(0.1) {
    p_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    filtered_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  }

  ~PointCloudMap() {
  }

/////////
  void readMapFile(string filename) {
    pcl::io::loadPCDFile("filename", *p_cloud);
  }

  void filterMap() {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(LeafSize, LeafSize, LeafSize);
    approximate_voxel_filter.setInputCloud(p_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);

    ROS_INFO("[PointCloudMap::filtterMap] source_cloud point num = %d", source_cloud->points.size());
    ROS_INFO("[PointCloudMap::filtterMap] filtered_cloud point num = %d", filtered_cloud->points.size());
  }

};

#endif
