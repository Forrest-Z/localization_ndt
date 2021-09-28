#ifndef LOCALI_LAUNCHER_H_
#define LOCALI_LAUNCHER_H_

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <mutex> // 排他制御

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>

#include "MyUtil.h"
#include "FrontEnd.h"
#include "PointCloudMap.h"
//#include "PoseGraph.h"
#include "Timer.h"
#include "PoseEstimator.h"
//#include "DataAssociator.h"
//#include "CostFunction.h"
//#include "PoseFuser.h"
//#include "NNGridTable.h"

/////////

class LocalizationLauncher{
private:
  std::string map_filename;

  Pose2D start;
  double start_x, start_y, start_th;

  enum state {
    Init,
    Esti
  }
  enum state st; // 状態

  Pose2D lidarOffset_front;         // レーザスキャナとロボットの相対位置
  Pose2D lidarOffset_left;          // レーザスキャナとロボットの相対位置
  Pose2D lidarOffset_right;         // レーザスキャナとロボットの相対位置

  PointCloudMap pcmap;              // 点群地図
  PoseEstimator estim;           // ロボット位置推定器

  Timer timer;

  int hz;     // メインループのレート
  int stamp;    // タイムスタンプ

  Scan2D scan;  // 最新のスキャン

  ros::Subscriber sub_odom, sub_scan;
  ros::Publisher pub_pc, pub_poseArray;

  std::mutex m;

public:
  SlamLauncher() : hz(2), stamp(0), st(Init),
                   start_x(0), start_y(0), start_th(0)
  {
    ros::NodeHandle nh;
    sub_odom = nh.subscribe("odom", 1, &SlamLauncher::cb_odom, this);
    sub_scan = nh.subscribe("scan", 1, &SlamLauncher::cb_scan, this);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    ros::param::get("main_hz", hz);
    ros::param::get("draw_skip", drawSkip);
    ros::param::get("map_filename", map_filename);

    ros::param::get("start_x", start_x);
    ros::param::get("start_y", start_y);
    ros::param::get("start_th", start_th);

    // 初期姿勢を設定
    start.tx = start_x;
    start.ty = start_y;
    start.th = start_th;
    start.calRmat();
  }

  ~SlamLauncher() {
  }

///////////
  void cb_odom(const nav_msgs::Odometry &odom_msg);
  void cb_scan(const sensor_msgs::LaserScan &scan_msg);
  void init();
  void main_loop();
};

#endif
