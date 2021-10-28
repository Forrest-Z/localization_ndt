#include "localization_ndt/ScanMatcher.h"

// 初期値initPoseを与えて、NDTによりロボット位置の推定値estPoseを求める
double ScanMatcher::matchScan(Pose2D &initPose, Pose2D &estPose) {
  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation(DEG2RAD(initPose.th), Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation(initPose.tx, initPose.ty, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  timer.start_timer();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, init_guess);
  Eigen::Matrix4f T = ndt.getFinalTransformation();

  timer.end_timer();
  ROS_INFO("align");
  timer.print_timer();

  double prob = ndt.getTransformationProbability();

  ROS_INFO("[ScanMatcher::matchScan] prob = %f", prob);
  ROS_INFO("[ScanMatcher::matchScan] FinalNumIteration = %d", ndt.getFinalNumIteration());

  double theta;
  if (T(0,0) > 0 && T(1,0) > 0) theta = std::asin(T(1,0));
  else if (T(0,0) > 0 && T(1,0) < 0) theta = std::asin(T(1,0));
  else if (T(0,0) < 0 && T(1,0) > 0) theta = std::acos(T(0,0));
  else theta = std::acos(T(0,0))*(-1.0);
  estPose.setPose(T(0, 3), T(1, 3), RAD2DEG(theta));

  double score = ndt.getFitnessScore();
  ROS_INFO("[ScanMatcher::matchScan] score = %f", score);

  if (ndt.hasConverged() == 0) { // 収束してなかったら失敗
    ROS_INFO("[ScanMatcher::matchScan] hasConverged:false");
  } else {
    ROS_INFO("[ScanMatcher::matchScan] hasConverged:true");
  }

  return score;
}
