#include <ros/ros.h>
#include "mpc_handle.hpp"

// ROS Msgs
#include "geometry_msgs/PolygonStamped.h"

namespace ns_mpc {

MPCHandle::MPCHandle(ros::NodeHandle &nodeHandle) :
  nodeHandle_(nodeHandle),
  mpc_(nodeHandle) {

  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
  mpc_.setMaxSpeed(max_speed_);
}

// Getters
int MPCHandle::getNodeRate() const {
  return node_rate_;
}

// Methods
void MPCHandle::loadParameters() {
    ROS_INFO("loading MPC handle parameters");

    if (!nodeHandle_.param("max_speed", max_speed_, 3.0)) {
        ROS_WARN_STREAM("Did not load max_speed. Standard value is: " << max_speed_);
    }

    if (!nodeHandle_.param<std::string>("slam_state_topic_name",
                                        slam_state_topic_name_,
                                        "/estimation/slam/state")) {
        ROS_WARN_STREAM("Did not load slam_state_topic_name. Standard value is: " << slam_state_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("velocity_estimate_topic_name",
                                        velocity_estimate_topic_name_,
                                        "/estimation/velocity_estimation/velocity_estimate")) {
        ROS_WARN_STREAM("Did not load velocity_estimate_topic_name. Standard value is: " << velocity_estimate_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("center_line_topic_name",
                                        center_line_topic_name_,
                                        "/control/mpc/center_line")) {
        ROS_WARN_STREAM("Did not load center_line_topic_name. Standard value is: " << center_line_topic_name_);
    }

    if (!nodeHandle_.param<std::string>("control_command_topic_name",
                                        control_command_topic_name_,
                                        "/control/mpc/car_command")) {
        ROS_WARN_STREAM("Did not load control_command_topic_name. Standard value is: " << control_command_topic_name_);
    }

    if (!nodeHandle_.param("node_rate", node_rate_, 20)) {
        ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
    }
}

void MPCHandle::subscribeToTopics() {
    ROS_INFO("subscribe to MPC topics");

    slamStateSubscriber_ = nodeHandle_.subscribe(
        slam_state_topic_name_, 1, &MPCHandle::slamStateCallback, this);

    velocityEstimateSubscriber_ = nodeHandle_.subscribe(
        velocity_estimate_topic_name_, 1, &MPCHandle::velocityEstimateCallback, this);
}

void MPCHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  controlCommandPublisher_ = nodeHandle_.advertise<fsd_common_msgs::ControlCommand>(control_command_topic_name_, 1);
  centerLinePublisher_     = nodeHandle_.advertise<geometry_msgs::PolygonStamped>(center_line_topic_name_, 1, true);
}

void MPCHandle::run() {
  mpc_.runAlgorithm();
  sendControlCommand();
}

void MPCHandle::sendControlCommand() {
  control_command_.throttle = mpc_.getControlCommand().throttle;
  control_command_.steering_angle = mpc_.getControlCommand().steering_angle;
  control_command_.header.stamp = ros::Time::now();
  controlCommandPublisher_.publish(control_command_);
}

void MPCHandle::slamMapCallback(const fsd_common_msgs::Map &map) {
    geometry_msgs::PolygonStamped center_line;

    { // Find Center Line (yellow-blue cone midpoint)
        center_line.polygon.points.clear();
        for (const auto &yellow : map.cone_yellow) {
            const auto it_blue = std::min_element(map.cone_blue.begin(), map.cone_blue.end(),
                                                  [&](const fsd_common_msgs::Cone &a,
                                                      const fsd_common_msgs::Cone &b) {
                                                      const double da = std::hypot(yellow.position.x - a.position.x,
                                                                                   yellow.position.y - a.position.y);
                                                      const double db = std::hypot(yellow.position.x - b.position.x,
                                                                                   yellow.position.y - b.position.y);
                                                      return da < db;
                                                  });

            geometry_msgs::Point32 p;
            p.x = static_cast<float>((yellow.position.x + it_blue->position.x) / 2.0);
            p.y = static_cast<float>((yellow.position.y + it_blue->position.y) / 2.0);
            p.z = 0.0;
            center_line.polygon.points.push_back(p);
        }
    }

    geometry_msgs::Polygon dense_center_line;
    { // Densify the center line
        const double precision = 0.2;
        for (unsigned int i = 1; i < center_line.polygon.points.size(); i++) {
            const double dx = center_line.polygon.points[i].x - center_line.polygon.points[i - 1].x;
            const double dy = center_line.polygon.points[i].y - center_line.polygon.points[i - 1].y;
            const double d  = std::hypot(dx, dy);

            const int nm_add_points = d / precision;
            for (unsigned int j = 0; j < nm_add_points; ++j) {
                geometry_msgs::Point32 new_p = center_line.polygon.points[i - 1];
                new_p.x += precision * j * dx / d;
                new_p.y += precision * j * dy / d;
                dense_center_line.points.push_back(new_p);
            }
        }
    }

    center_line.polygon         = dense_center_line;
    center_line.header.frame_id = "map";
    center_line.header.stamp    = ros::Time::now();
    centerLinePublisher_.publish(center_line);

    mpc_.setCenterLine(dense_center_line);
}

void MPCHandle::slamStateCallback(const fsd_common_msgs::CarState &state) {
  mpc_.setState(state);
}
void MPCHandle::velocityEstimateCallback(const fsd_common_msgs::CarStateDt &velocity) {
  mpc_.setVelocity(velocity);
}
} // namespace ns_mpc