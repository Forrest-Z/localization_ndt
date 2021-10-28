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
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "MyUtil.h"
#include "PointCloudMap.h"
#include "Timer.h"
#include "PoseEstimator.h"

/////////

class LocalizationLauncher{
private:
  std::string map_filename;

  Pose2D start;
  double start_x, start_y, start_th;

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
  ros::Publisher pub_pose;

  tf2_ros::TransformBroadcaster dynamic_br;
  std_msgs::Header header;
  std::string odom_topic_name, scan_topic_name;

  std::mutex m;

public:
  LocalizationLauncher() : hz(2), stamp(0),
                           start_x(0), start_y(0), start_th(0),
                           odom_topic_name("/ypspur_ros/odom"), scan_topic_name("/scan")
  {
    ros::NodeHandle nh;

    ros::param::get("odom_topic_name", odom_topic_name);
    ros::param::get("scan_topic_name", scan_topic_name);

    sub_odom = nh.subscribe(odom_topic_name, 1, &LocalizationLauncher::cb_odom, this);
    sub_scan = nh.subscribe(scan_topic_name, 1, &LocalizationLauncher::cb_scan, this);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    ros::param::get("main_hz", hz);
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

  ~LocalizationLauncher() {
  }

///////////
  void cb_odom(const nav_msgs::Odometry &odom_msg);
  void cb_scan(const sensor_msgs::LaserScan &scan_msg);
  void init();
  void publishTF(const Pose2D &pose2d);
  void publishPose(const Pose2D &pose2d);
  void main_loop();
};

#endif
