#ifndef POINT_CLOUD_MAP_H_
#define POINT_CLOUD_MAP_H_

using namespace std;

#include <ros/ros.h>

#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <boost/make_shared.hpp>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "Timer.h"

class PointCloudMap {
private:
  static const int MAX_POINT_NUM=10000000;             // globalMapの最大点数

  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud;         // ファイル入力用

  double LeafSize; // フィルタサイズ

public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;  // フィルター済み地図

  PointCloudMap() : LeafSize(0.1) {
    ros::param::get("LeafSize", LeafSize);

    p_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filtered_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  ~PointCloudMap() {
  }

/////////
  void readMapFile(string filename) {
    pcl::io::loadPCDFile(filename, *p_cloud);
    if (p_cloud->points.size() > MAX_POINT_NUM) {
      ROS_INFO("[PointCloudMap::readMapFile] MAX_POINT_NUM Error");
    }
    ROS_INFO("[PointCloudMap::readMapFile] OK.");
  }

/////////
  void filterMap();

};

#endif
