#include <ros/ros.h>
#include "mpc.hpp"

#include <sstream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <casadi/casadi.hpp>

// ROS Msgs
#include "visualization_msgs/MarkerArray.h"

namespace ns_mpc {

// Constructor
MPC::MPC(ros::NodeHandle &nh) : nh_(nh) {
    pub_closest_point_ = nh.advertise<visualization_msgs::MarkerArray>("/control/mpc/marker", 1);

    if (!nh.param<double>("controller/speed/p", speed_p, 0.01)) {
        ROS_WARN_STREAM("Did not load controller/speed/p. Standard value is: " << 0.01);
    }
    if (!nh.param<double>("controller/steering/p", steering_p, 0.01)) {
        ROS_WARN_STREAM("Did not load controller/steering/p. Standard value is: " << 0.01);
    }
};

// Getters
fsd_common_msgs::ControlCommand MPC::getControlCommand() const { return control_command_; }

// Setters
void MPC::setMaxSpeed(double &max_speed) {
    max_speed_ = max_speed;
}

void MPC::setCenterLine(const geometry_msgs::Polygon &center_line) {
    center_line_ = center_line;
}

void MPC::setState(const fsd_common_msgs::CarState &state) {
    state_ = state;
}

void MPC::setVelocity(const fsd_common_msgs::CarStateDt &velocity) {
    velocity_ = velocity;
}

std::vector<std::pair<double, double>> getReferenceTrajectory(
    const std::vector<geometry_msgs::Point32>& center_line,
    const double current_x,
    const double current_y,
    const int N,
    const int gap = 2)
{
    std::vector<std::pair<double, double>> ref;

    if (center_line.empty()) return ref;

    // 1. 현재 위치에서 가장 가까운 점 찾기
    const auto it_closest = std::min_element(center_line.begin(), center_line.end(),
        [&](const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b) {
            const double da = std::hypot(current_x - a.x, current_y - a.y);
            const double db = std::hypot(current_x - b.x, current_y - b.y);
            return da < db;
        });

    const int closest_idx = std::distance(center_line.begin(), it_closest);
    const int cl_size = center_line.size();

    // 2. 일정 간격으로 N개 점 추출
    for (int i = 0; i < N; ++i) {
        int idx = closest_idx + i * gap;
        if (idx >= cl_size) idx = cl_size - 1;

        ref.emplace_back(center_line[idx].x, center_line[idx].y);
    }

    return ref;
}

void MPC::runAlgorithm() {
    if (center_line_.points.empty()) {
        control_command_.throttle = -1.0;
        control_command_.steering = 0.0;
        return;
    }

    using namespace casadi;

    const int N = 10;
    const double dt = 0.1;
    const double L = 1.33;
    const double max_steer = 0.5;
    const double max_acc = 3.0;

    // Define state and control variables
    SX x = SX::sym("x"), y = SX::sym("y"), theta = SX::sym("theta"), v = SX::sym("v");
    SX steer = SX::sym("steer"), a = SX::sym("a");
    SX state = vertcat({x, y, theta, v});
    SX control = vertcat({steer, a});

    SX rhs = vertcat({
        v * cos(theta),
        v * sin(theta),
        v * tan(steer) / L,
        a
    });
    Function f = Function("f", {state, control}, {rhs});

    SX X = SX::sym("X", 4, N + 1);
    SX U = SX::sym("U", 2, N);

    DM x0 = DM::zeros(4);
    x0(0) = state_.x;
    x0(1) = state_.y;
    x0(2) = state_.theta;
    x0(3) = velocity_.velocity;

    auto ref_traj = getReferenceTrajectory(center_line_.points, state_.x, state_.y, N);
    std::vector<double> x_ref(N), y_ref(N);
    for (int i = 0; i < N; ++i) {
        x_ref[i] = ref_traj[i].first;
        y_ref[i] = ref_traj[i].second;
    }

    SX cost = 0;
    for (int k = 0; k < N; ++k) {
        SX e_x = X(0, k) - x_ref[k];
        SX e_y = X(1, k) - y_ref[k];
        cost += e_x * e_x + e_y * e_y + 0.1 * U(0, k) * U(0, k) + 0.1 * U(1, k) * U(1, k);
    }

    std::vector<SX> g;
    g.push_back(X(Slice(), 0) - x0);
    for (int k = 0; k < N; ++k) {
        SX x_next = X(Slice(), k) + dt * f(X(Slice(), k), U(Slice(), k))[0];
        g.push_back(X(Slice(), k + 1) - x_next);
    }

    SXDict nlp = {
        {"x", vertcat({reshape(X, 4 * (N + 1), 1), reshape(U, 2 * N, 1)})},
        {"f", cost},
        {"g", vertcat(g)}
    };

    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;

    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    std::vector<double> lbx(4 * (N + 1) + 2 * N, -inf);
    std::vector<double> ubx(4 * (N + 1) + 2 * N, inf);
    for (int k = 0; k < N; ++k) {
        lbx[4 * (N + 1) + 2 * k] = -max_steer;
        ubx[4 * (N + 1) + 2 * k] = max_steer;
        lbx[4 * (N + 1) + 2 * k + 1] = -max_acc;
        ubx[4 * (N + 1) + 2 * k + 1] = max_acc;
    }

    DMDict arg;
    arg["x0"] = DM::zeros(4 * (N + 1) + 2 * N);
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = std::vector<double>(g.size() * 4, 0);
    arg["ubg"] = std::vector<double>(g.size() * 4, 0);

    DMDict res = solver(arg);
    DM sol = res["x"];
    DM U_opt = sol(Slice(4 * (N + 1), sol.size1()));

    control_command_.steering = static_cast<float>(U_opt(0));
    control_command_.throttle = static_cast<float>(U_opt(1));

    // Visualize
    publishMarkers(it_center_line->x, it_center_line->y, next_point.x, next_point.y);
}

// void MPC::createControlCommand() {
//     // 1. 가장 가까운 점 찾기
//     // 2. 다음 목표점 설정
//     // 3. Pure Pursuit 식 비슷하게 조향각 계산 (eta, atan 사용)
//     // 4. 현재 속도와 max_speed 비교해서 throttle 계산

// }

void MPC::publishMarkers(double x_pos, double y_pos, double x_next, double y_next) const {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker marker;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = y_pos;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    markers.markers.push_back(marker);

    marker.pose.position.x = x_next;
    marker.pose.position.y = y_next;
    marker.color.b = 1.0;
    marker.id = 1;
    markers.markers.push_back(marker);

    pub_closest_point_.publish(markers);
}

} // namespace ns_mpc