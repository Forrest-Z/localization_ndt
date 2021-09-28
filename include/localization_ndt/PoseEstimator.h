#ifndef POSEESTIMATOR_ICP_H_
#define POSEESTIMATOR_ICP_H_

#include <ros/ros.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "Timer.h"

//////

class PoseEstimator{
private:
  PointCloudMap *pcmap;              // 点群地図
  ScanMatcher2D smat;                // スキャンマッチング

  Pose2D initPose;         // 初期姿勢
  Pose2D lastPose;         // 直前姿勢

  Scan2D lastScan;         // 直前スキャン

public:
  void setPointCloudMap(PointCloudMap *pcmap_) {
    pcmap = pcmap_;
    smat.setPointCloudMap(pcmap_);
  }

  void setInitPose(const Pose2D &pose) {
    initPose.tx = pose.tx;
    initPose.ty = pose.ty;
    initPose.th = pose.th;
    initPose.calRmat();
  }

  Pose2D getLastPose() {
    return lastPose;
  }
/////////
  void estimetaInitPose(Scan2D &curScan);
  void estimatePose(Scan2D &curScan);

};

#endif
