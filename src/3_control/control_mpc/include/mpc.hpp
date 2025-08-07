#ifndef CONTROL_MPC_HPP
#define CONTROL_MPC_HPP

#include "ros/ros.h"

#include <geometry_msgs/Polygon.h>
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

namespace ns_mpc {

class MPC {

 public:
    // Constructor
    MPC(ros::NodeHandle& nh);

    // Getters
    fsd_common_msgs::ControlCommand getControlCommand() const;

    // Setters
    void setMaxSpeed(double &max_speed);
    void setCenterLine(const geometry_msgs::Polygon &center_line);
    void setState(const fsd_common_msgs::CarState &state);
    void setVelocity(const fsd_common_msgs::CarStateDt &velocity);

    // Main processing
    void createControlCommand();
    void runAlgorithm();

 private:
 
    /**
     * Visualize
     */
    void publishMarkers(double x_pos, double y_pos, double x_next, double y_next) const;

    ros::NodeHandle& nh_;
    ros::Publisher pub_closest_point_;

    double speed_p;
    double steering_p;

    geometry_msgs::Polygon          center_line_;
    fsd_common_msgs::CarState       state_;
    fsd_common_msgs::CarStateDt     velocity_;
    fsd_common_msgs::ControlCommand control_command_;

    double max_speed_;
};
}

#endif // CONTROL_MPC_HPP