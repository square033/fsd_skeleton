/*
    Formula Student Driverless Project (FSD-Project).
    GPLv3
*/

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

/*  ---- 주석 처리: 더 이상 사용하지 않음 (Pure Pursuit와 동일 로직으로 대체) ----
std::vector<std::pair<double, double>> getReferenceTrajectory(
    const std::vector<geometry_msgs::Point32>& center_line,
    const double current_x,
    const double current_y,
    const int N,
    const int gap)
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
    const int cl_size = static_cast<int>(center_line.size());

    // 2. 일정 간격으로 N개 점 추출
    for (int i = 0; i < N; ++i) {
        int idx = closest_idx + i * gap;
        if (idx >= cl_size) idx = cl_size - 1;
        ref.emplace_back(center_line[idx].x, center_line[idx].y);
    }

    return ref;
}
---- 주석 끝 ---- */

void MPC::runAlgorithm() {
    if (center_line_.points.empty()) {
        control_command_.throttle.data = -1.0f;
        control_command_.steering_angle.data = 0.0f;
        return;
    }

    using namespace casadi;

    const int N = 10;
    const double dt = 0.1;
    const double L = 1.33;
    const double max_steer = 0.5;
    const double max_acc   = 0.2;

    // Define state and control variables
    SX x = SX::sym("x"), y = SX::sym("y"), theta = SX::sym("theta"), v = SX::sym("v");
    SX steer = SX::sym("steer"), a = SX::sym("a");
    SX state(4,1);   state(0)=x;      state(1)=y;        state(2)=theta;        state(3)=v;
    SX control(2,1); control(0)=steer; control(1)=a;

    SX rhs(4,1);
    rhs(0) = v * cos(theta);
    rhs(1) = v * sin(theta);
    rhs(2) = (v / L) * tan(steer);
    rhs(3) = a;

    Function f = Function("f", {state, control}, {rhs});

    // Decision variables
    SX X = SX::sym("X", 4, N + 1);
    SX U = SX::sym("U", 2, N);

    // Initial state from messages
    DM x0 = DM::zeros(4);
    x0(0) = state_.car_state.x;
    x0(1) = state_.car_state.y;
    x0(2) = state_.car_state.theta;
    x0(3) = velocity_.car_state_dt.x; // 필요시: hypot(vx,vy)로 교체 가능

    // -------------------------------
    // Pure Pursuit와 동일한 목표점 선택:
    // 최근접 인덱스 + 고정 오프셋(lookahead), 모듈로 래핑
    // -------------------------------
    const auto it_closest = std::min_element(center_line_.points.begin(), center_line_.points.end(),
        [&](const geometry_msgs::Point32 &a, const geometry_msgs::Point32 &b) {
            const double da = std::hypot(state_.car_state.x - a.x, state_.car_state.y - a.y);
            const double db = std::hypot(state_.car_state.x - b.x, state_.car_state.y - b.y);
            return da < db;
        });
    const int i_center  = static_cast<int>(std::distance(center_line_.points.begin(), it_closest));
    const int size      = static_cast<int>(center_line_.points.size());
    const int lookahead = 10; // Pure Pursuit의 +10과 동일
    const int i_next    = (i_center + lookahead) % size;

    geometry_msgs::Point32 next_point = center_line_.points[i_next];

    // PP처럼 하나의 목표점을 N 스텝 동안 반복해서 따라가게 설정
    std::vector<double> x_ref(N, next_point.x);
    std::vector<double> y_ref(N, next_point.y);
    // -------------------------------

    // Cost
    SX cost = 0;
    for (int k = 0; k < N; ++k) {
        SX e_x = X(0, k) - x_ref[k];
        SX e_y = X(1, k) - y_ref[k];
        cost += e_x * e_x + e_y * e_y
              + 0.1 * U(0, k) * U(0, k)  // steer penalty
              + 0.1 * U(1, k) * U(1, k); // accel penalty
    }

    // ---- Build constraints and flatten variables (no vertcat/reshape) ----
    // Flatten X, U
    SX Xvec(4 * (N + 1), 1);
    for (int j = 0; j <= N; ++j)
        for (int i = 0; i < 4; ++i)
            Xvec(4 * j + i) = X(i, j);

    SX Uvec(2 * N, 1);
    for (int j = 0; j < N; ++j) {
        Uvec(2 * j + 0) = U(0, j);
        Uvec(2 * j + 1) = U(1, j);
    }

    // z = [vec(X); vec(U)]
    SX z(4 * (N + 1) + 2 * N, 1);
    for (int r = 0; r < 4 * (N + 1); ++r) z(r) = Xvec(r);
    for (int r = 0; r < 2 * N;       ++r) z(4 * (N + 1) + r) = Uvec(r);

    // Constraint vector G = 0 (size 4*(N+1))
    SX x0_sym = SX::sym("x0_sym", 4);
    SX G(4 * (N + 1), 1);

    // Initial constraint: X(:,0) - x0
    for (int i = 0; i < 4; ++i) G(i) = X(i, 0) - x0_sym(i);

    // Dynamics constraints: X(:,k+1) - (X(:,k) + dt*f(...)) = 0
    for (int k = 0; k < N; ++k) {
        SX x_next = X(casadi::Slice(), k) + dt * f(std::vector<SX>{X(casadi::Slice(), k), U(casadi::Slice(), k)})[0];
        for (int i = 0; i < 4; ++i) G(4 * (k + 1) + i) = X(i, k + 1) - x_next(i);
    }

    // NLP dict
    SXDict nlp;
    nlp["x"] = z;
    nlp["f"] = cost;
    nlp["g"] = G;
    nlp["p"] = x0_sym;

    // Solver options
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;

    Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Bounds
    double inf = std::numeric_limits<double>::infinity();
    std::vector<double> lbx(4 * (N + 1) + 2 * N, -inf);
    std::vector<double> ubx(4 * (N + 1) + 2 * N,  inf);
    for (int k = 0; k < N; ++k) {
        // steering bounds
        lbx[4 * (N + 1) + 2 * k + 0] = -max_steer;
        ubx[4 * (N + 1) + 2 * k + 0] =  max_steer;
        // accel bounds
        lbx[4 * (N + 1) + 2 * k + 1] = -max_acc;
        ubx[4 * (N + 1) + 2 * k + 1] =  max_acc;
    }

    // Solve
    DMDict arg;
    arg["x0"]  = DM::zeros(4 * (N + 1) + 2 * N);
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = std::vector<double>(4 * (N + 1), 0.0);
    arg["ubg"] = std::vector<double>(4 * (N + 1), 0.0);
    arg["p"]   = x0;

    DMDict res = solver(arg);
    DM sol = res["x"];

    // Extract first control (u0)
    const int offU = 4 * (N + 1);
    DM U_opt = sol(casadi::Slice(offU, offU + 2 * N));
    const double steer_cmd    = static_cast<double>(U_opt(0));
    const double throttle_cmd = static_cast<double>(U_opt(1));

    control_command_.steering_angle.data = static_cast<float>(steer_cmd);
    control_command_.throttle.data       = static_cast<float>(throttle_cmd);

    // Visualize: Pure Pursuit와 동일하게 "최근접점 vs 다음점"
    publishMarkers(it_closest->x, it_closest->y, next_point.x, next_point.y);
}

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