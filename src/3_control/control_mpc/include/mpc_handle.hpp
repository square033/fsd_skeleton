#ifndef CONTROL_MPC_HANDLE_HPP
#define CONTROL_MPC_HANDLE_HPP

#include <ros/ros.h>
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "geometry_msgs/PolygonStamped.h"
#include "mpc.hpp"

namespace ns_mpc {

class MPCHandle {

 public:
  // Constructor
  explicit MPCHandle(ros::NodeHandle &nodeHandle);

  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendControlCommand();

 private:
  ros::NodeHandle nodeHandle_;

  ros::Publisher controlCommandPublisher_;
  ros::Publisher centerLinePublisher_;

  ros::Subscriber slamStateSubscriber_;
  ros::Subscriber velocityEstimateSubscriber_;
  ros::Subscriber centerLineSubscriber_;

  // Callbacks
  void slamMapCallback(const fsd_common_msgs::Map &map);
  void slamStateCallback(const fsd_common_msgs::CarState &state);
  void velocityEstimateCallback(const fsd_common_msgs::CarStateDt &velocity);

  // Parameters
  std::string slam_state_topic_name_;
  std::string velocity_estimate_topic_name_;
  std::string center_line_topic_name_;
  std::string control_command_topic_name_;

  int node_rate_;
  double max_speed_;

  MPC mpc_;
  fsd_common_msgs::ControlCommand control_command_;
};
}

#endif // CONTROL_MPC_HANDLE_HPP