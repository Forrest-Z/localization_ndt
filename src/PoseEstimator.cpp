#include "localization_ndt/PoseEstimator.h"

bool PoseEstimator::estimetaInitPose(Scan2D &curScan) {
  // 参照スキャンを登録
  smat.setMap(initPose);

  // 現在スキャンの登録
  smat.setCurScan(curScan.lps);

  // スキャンマッチによる推定位置
  Pose2D estPose;
/*
  double min_score = 100000000000;
  for (double dx = -3.0; dx <= 3.0; dx+=0.5) {
    for (double dy = -3.0; dy <= 3.0; dy+=0.5) {
      for (double dth = -5.0; dth <= 5.0; dth+=2.0) {
        Pose2D tempPose(initPose.tx+dx, initPose.ty+dy, initPose.th+dth);
        Pose2D tempEstPose;
        double score = smat.matchScan(tempPose, tempEstPose); // スキャンマッチ
        ROS_INFO("[PoseEstimator::estimatePose] tempEstPose: tx=%f, ty=%f, th=%f\n"
            , tempEstPose.tx, tempEstPose.ty, tempEstPose.th);
        if (score < min_score) {
          min_score = score;
          estPose = tempEstPose;
        }
      }
    }
  }

  ROS_INFO("[PoseEstimator::estimatePose] initPose: tx=%f, ty=%f, th=%f\n"
            , initPose.tx, initPose.ty, initPose.th);
  ROS_INFO("[PoseEstimator::estimatePose] estPose: tx=%f, ty=%f, th=%f\n"
            , estPose.tx, estPose.ty, estPose.th);

  // 閾値より小さければ(大きければ？)成功とする
  bool successful;                             // スキャンマッチングに成功したかどうか
  if (min_score < scthre) successful = true;
  else successful = false;

  ROS_INFO("[PoseEstimator::estimatePose] min_score=%g, scthre=%g, successful=%d", min_score, scthre, successful);
*/
  estPose = initPose;

  lastScan = curScan;  // 直前スキャンを設定
  lastPose = estPose;  // 直前姿勢を設定

//  return successful;
  return true;
}

bool PoseEstimator::estimatePose(Scan2D curScan) {
  ROS_INFO("[PoseEstimator::estimatePose] curScan.sid = %d", curScan.sid);
  // オドメトリから初期値を設定
  // Scanに入っているオドメトリ値を用いて移動量を計算する
  // 移動量 + 1つ前の自己位置 = 現在の自己位置(NDTの初期値)
  Pose2D odoMotion;                                                 // オドメトリに基づく移動量
  Pose2D::calMotion(curScan.pose, lastScan.pose, odoMotion);        // 前スキャンとの相対位置が移動量
  Pose2D predPose;                                                  // オドメトリによる予測位置
  Pose2D::calPredPose(odoMotion, lastPose, predPose);               // 直前位置に移動量を加えて予測位置を得る

  ROS_INFO("[PoseEstimator::estimatePose] odoMotion: tx=%f, ty=%f, th=%f\n"
            , odoMotion.tx, odoMotion.ty, odoMotion.th);
  ROS_INFO("[PoseEstimator::estimatePose] lastPose: tx=%f, ty=%f, th=%f\n"
            , lastPose.tx, lastPose.ty, lastPose.th);
  ROS_INFO("[PoseEstimator::estimatePose] predPose: tx=%f, ty=%f, th=%f\n"
            , predPose.tx, predPose.ty, predPose.th);

  // 参照スキャンを登録
  smat.setMap(predPose);

  // 現在スキャンの登録
  smat.setCurScan(curScan.lps);

  // スキャンマッチによる推定位置
  Pose2D estPose;
  double score = smat.matchScan(predPose, estPose); // スキャンマッチ

  ROS_INFO("[PoseEstimator::estimatePose] estPose: tx=%f, ty=%f, th=%f\n"
            , estPose.tx, estPose.ty, estPose.th);

  // 閾値より小さければ(大きければ？)成功とする
  bool successful;                             // スキャンマッチングに成功したかどうか
  if (score <= scthre) successful = true;
  else successful = false;

  ROS_INFO("[PoseEstimator::estimatePose] score=%g, scthre=%g, successful=%d", score, scthre, successful);

  // 成功でなければ,オドメトリによる予測位置を使う
  if (!successful){
    estPose = predPose;
  }

  lastScan = curScan;  // 直前スキャンを設定
  lastPose = estPose;  // 直前姿勢を設定

  return successful;
}
