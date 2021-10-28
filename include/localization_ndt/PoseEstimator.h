#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include <ros/ros.h>
#include <vector>

#include <pcl/point_types.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "Timer.h"
#include "PointCloudMap.h"
#include "ScanMatcher.h"

//////

class PoseEstimator {
private:
  PointCloudMap *pcmap;            // 点群地図
  ScanMatcher smat;                // スキャンマッチング

  Pose2D initPose;         // 初期姿勢
  Pose2D lastPose;         // 直前姿勢

  Scan2D lastScan;         // 直前スキャン

  double scthre;           // スコアしきい値

public:
  PoseEstimator() : scthre(0.5) {
    ros::param::get("scthre", scthre);
  }

  ~PoseEstimator() {}

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
  bool estimetaInitPose(Scan2D &curScan);
  bool estimatePose(Scan2D curScan);

};

#endif
