#ifndef SCAN_MATCHER2D_H_
#define SCAN_MATCHER2D_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <vector>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"
#include "RefScanMaker.h"
#include "ScanPointResampler.h"
//#include "ScanPointAnalyser.h"
#include "PoseEstimator.h"
//#include "PoseFuser.h"
#include "TFBroadcaster.h"
#include "Timer.h"

class ScanMatcher {
private:
  PointCloudMap *pcmap;

  double TransformationEpsilon;
  double StepSize;
  double Resolution;
  int MaximumIterations;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud; // 現在スキャン
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud; // 参照マップ

public:
  ScanMatcher() : TransformationEpsilon(0.01), StepSize(0.1), Resolution(1.0), MaximumIterations(35) {
    ros::param::get("TransformationEpsilon", TransformationEpsilon);
    ros::param::get("StepSize", StepSize);
    ros::param::get("Resolution", Resolution);
    ros::param::get("MaximumIterations", MaximumIterations);

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
  void setMap() {
    target_cloud = pcmap->filtered_cloud;
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
  }

};
#endif
