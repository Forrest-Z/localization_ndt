#include "localization_ndt/PointCloudMap.h"

void PointCloudMap::filterMap() {
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(LeafSize, LeafSize, LeafSize);
  approximate_voxel_filter.setInputCloud(p_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);

  ROS_INFO("[PointCloudMap::filtterMap] p_cloud point num = %d", p_cloud->points.size());
  ROS_INFO("[PointCloudMap::filtterMap] filtered_cloud point num = %d", filtered_cloud->points.size());
}
