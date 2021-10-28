#include <ros/ros.h>

#include "localization_ndt/LocalizationLauncher.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "localization_ndt");

  LocalizationLauncher ll;
  ll.init();
  ll.main_loop();

  ROS_INFO("End.");

  return 0;
}
