#include "localization_ndt/LocalizationLauncher.h"

void LocalizationLauncher::init() {
  pcmap.readMapFile(map_filename);
  pcmap.filterMap();

  estim.setPointCloudMap(&pcmap);
  estim.setInitPose(start);
}

void LocalizationLauncher::cb_odom(const nav_msgs::Odometry &odom_msg){
  std::lock_guard<std::mutex> lock(m);
  double roll, pitch, yaw;

  scan.pose.tx = odom_msg.pose.pose.position.x;
  scan.pose.ty = odom_msg.pose.pose.position.y;
  MyUtil::geometry_quat_to_rpy(roll, pitch, yaw, odom_msg.pose.pose.orientation);
  scan.pose.th = RAD2DEG(yaw);
  scan.pose.calRmat();

  scan.sid = stamp;
}

void LocalizationLauncher::cb_scan(const sensor_msgs::LaserScan &scan_msg){
  std::lock_guard<std::mutex> lock(m);

  // 問題のある点(30m以上 もしくは inf)を探す
  int count = 0;         // 問題ない点の数
  double angle = -135.0; // スキャンの開始角度
  vector<bool> ok_range;
  for(int i = 0; i < scan_msg.ranges.size(); i++) {
    // -90度〜90度のみ使用 無限大は無視 30m以内のセンサ値のみ使用
    if(angle > -90 && angle < 90 &&
       std::isinf(scan_msg.ranges[i]) == 0 && scan_msg.ranges[i] < 30)
    {
      count++;
      ok_range.push_back(true);
    } else {
      ok_range.push_back(false);
    }
    angle += 0.25;
  }

  // 問題ない点をscanに格納
  scan.lps.clear();
  scan.lps.reserve(count);
  angle = -135.0;
  for(int i = 0; i < scan_msg.ranges.size(); i++) {
    LPoint2D lp;
    if(ok_range[i] == true) {
      lp.sid = stamp;
      lp.set_RangeAngle2XY(scan_msg.ranges[i], angle);
      scan.lps.push_back(lp);
    }
    angle += 0.25;
  }

  scan.sid = stamp;

  header = scan_msg.header;

  publishTF(estim.getLastPose());
}

void LocalizationLauncher::publishTF(const Pose2D &pose2d) {
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = header.stamp;
//  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.seq = stamp;
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "laser";
  transformStamped.transform.translation.x = pose2d.tx;
  transformStamped.transform.translation.y = pose2d.ty;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation = MyUtil::rpy_to_geometry_quat(0.0, 0.0, DEG2RAD(pose2d.th));
  dynamic_br.sendTransform(transformStamped);
}

void LocalizationLauncher::publishPose(const Pose2D &pose2d) {
// PoseStamped
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.pose.position.x = pose2d.tx;
  pose_stamped.pose.position.y = pose2d.ty;
  pose_stamped.pose.position.z = 0;
  pose_stamped.pose.orientation = MyUtil::rpy_to_geometry_quat(0.0, 0.0, DEG2RAD(pose2d.th));
  pose_stamped.header.frame_id = "map";
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.seq = stamp;

  pub_pose.publish(pose_stamped);
}

void LocalizationLauncher::main_loop() {
  ros::AsyncSpinner spinner(1);  // 引数:spinを処理するスレッド数
  spinner.start();

  // 初期姿勢を推定
  ros::Rate init_rate(0.5);
  init_rate.sleep();
  {
    std::lock_guard<std::mutex> lock(m);
    if (estim.estimetaInitPose(scan) == false) {
      ROS_INFO("[LocalizationLauncher::main_loop] estimetaInitPose : miss");
      return;
    }
    publishPose(estim.getLastPose());
  }

  ros::Rate rate(hz);
  Scan2D scan_copy;

  while(ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(m);
      scan_copy = scan;
    }
    ROS_INFO("--- [LocalizationLauncher::main_loop] stamp=%d ---", stamp);
    estim.estimatePose(scan_copy); // 処理本体
    publishPose(estim.getLastPose());
    stamp++;
    rate.sleep();
  }

  spinner.stop();

  return;
}
