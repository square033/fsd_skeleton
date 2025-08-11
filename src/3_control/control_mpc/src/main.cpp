/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2018:
     - Sonja Brits <britss@ethz.ch>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "mpc_handle.hpp"

typedef ns_mpc::MPCHandle MPCHandle;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpcController");
  ros::NodeHandle nodeHandle("~");

  MPCHandle mpcHandle(nodeHandle);
  ros::Rate loop_rate(mpcHandle.getNodeRate());

  while (ros::ok()) {
    mpcHandle.run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}