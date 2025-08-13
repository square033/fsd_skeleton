#include <ros/ros.h>
#include "mpc.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;

  ns_mpc::MPC mpc(nh);

  ros::Rate rate(20);
  while (ros::ok()) {
    mpc.runAlgorithm();   // 여기서 publishMarkers가 호출됨
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
