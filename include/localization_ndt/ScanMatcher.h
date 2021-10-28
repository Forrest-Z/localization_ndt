#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <vector>
#include <boost/make_shared.hpp>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"
#include "Timer.h"

class ScanMatcher {
private:
  PointCloudMap *pcmap;

  double TransformationEpsilon;
  double StepSize;
  double Resolution;
  int MaximumIterations;

  double LeafSize;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud; // 現在スキャン
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_filtered; // 現在スキャン(フィルター済み)
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud; // 参照マップ

  Timer timer;

public:
  ScanMatcher() : TransformationEpsilon(0.01), StepSize(0.1), Resolution(1.0), MaximumIterations(35), LeafSize(0.1) {
    ros::param::get("TransformationEpsilon", TransformationEpsilon);
    ros::param::get("StepSize", StepSize);
    ros::param::get("Resolution", Resolution);
    ros::param::get("MaximumIterations", MaximumIterations);

    ros::param::get("LeafSize", LeafSize);

    source_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    source_cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    target_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(TransformationEpsilon);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(StepSize);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(Resolution);
    // Setting max number of registration iterations.
    ndt.setMaximumIterations(MaximumIterations);
  }

  ~ScanMatcher() {
  }

///////
  void setPointCloudMap(PointCloudMap *pcmap_) {
    pcmap = pcmap_;
  }

  // 参照スキャンの登録
  void setMap(const Pose2D &center) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud = pcmap->filtered_cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(original_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(center.tx-20.0, center.tx+20.0);
    pass_x.filter(*xf_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(xf_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(center.ty-20.0, center.ty+20.0);
    pass_y.filter(*yf_cloud);

    target_cloud = yf_cloud;

    ndt.setInputTarget(target_cloud);

    ROS_INFO("[ScanMatcher::setMap] target_cloud->points.size = %d", target_cloud->points.size());
  }

  // 現在スキャンの登録
  void setCurScan(const std::vector<LPoint2D> &lps) {
    source_cloud->width = lps.size();
    source_cloud->height = 1;
    source_cloud->is_dense = false;
    source_cloud->points.resize(source_cloud->width * source_cloud->height);
    for (size_t i = 0; i < source_cloud->points.size(); i++) {
      source_cloud->points[i].x = lps[i].x;
      source_cloud->points[i].y = lps[i].y;
      source_cloud->points[i].z = 0;
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(LeafSize, LeafSize, LeafSize);
    approximate_voxel_filter.setInputCloud(source_cloud);
    approximate_voxel_filter.filter(*source_cloud_filtered);

    ndt.setInputSource(source_cloud_filtered);

    ROS_INFO("[ScanMatcher::setCurScan] source_cloud->points.size = %d", source_cloud->points.size());
    ROS_INFO("[ScanMatcher::setCurScan] source_cloud_filtered->points.size = %d", source_cloud_filtered->points.size());
  }

////////
  double matchScan(Pose2D &initPose, Pose2D &estPose);

};

#endif
